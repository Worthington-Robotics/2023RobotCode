package frc.robot.subsystems.arm;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.loops.ILooper;
import frc.lib.loops.Loop;
import frc.lib.util.HIDHelper;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Subsystem;
import frc.robot.subsystems.arm.ArmIO.ArmIOInputs;

public class Arm extends Subsystem {
    private static Arm instance = new Arm();
    private ArmNewIO periodic;
    public static Arm getInstance() {return instance;}

    private ArmIO io;
    private ArmTrajectoryManager trajectories;
	private ArmVisualizer visualizer;
    private ArmVisualizer setpointViz;

    // private PIDController shoulderController = new PIDController(12, 0, 0.5);
    // private PIDController extensionController = new PIDController(1.0, 0, 0.5);
    // private PIDController wristController = new PIDController(0.5, 0, 0.005);

    private PIDController shoulderController = new PIDController(.8, 0, 0.0);
    private PIDController extensionController = new PIDController(1.5, 0, 0.0);
    private PIDController wristController = new PIDController(.75, 0, 0.0);

    public Arm() {
        if(Robot.isSimulation()) {
            io = new ArmIOSim();
        } else {io = new ArmIOFalcon();}
        visualizer = new ArmVisualizer(Constants.Arm.ZERO_ANGLES, "Actual");
        setpointViz = new ArmVisualizer(ArmPose.Preset.ZERO.getAngles(), "Setpoint");
        trajectories = new ArmTrajectoryManager();
        periodic  = new ArmNewIO();
        periodic.timer.start();
    }

    public class ArmNewIO extends PeriodicIO {
        public ArmMode state = ArmMode.TRAJECTORY;
        public ArmIOInputs inputs = new ArmIOInputs();
        public Vector<N3> anglesSetpoint = VecBuilder.fill(0, 1, 0);

        public double openLoopShoulderJoyVal = 0;
        public double openLoopExtensionJoyVal = 0;
        public double openLoopWristJoyVal = 0;

        public double shoulderSetpoint = 0; //Shoulder motor setpoint in percent. 0.0 being 0% and 1 being 100%
        public double extensionSetpoint = 0; //Shoulder motor setpoint in percent. 0.0 being 0% and 1 being 100%
        public double wristSetpoint = 0; //Shoulder motor setpoint in percent. 0.0 being 0% and 1 being 100%

        public double manualX = 0;
        public double manualY = 0;

        public ArmTrajectory currentTrajectory = trajectories.getTrajectory(0);
        public ArmPose.Preset holdPose = ArmPose.Preset.ZERO;
        public boolean trajectoryComplete = true;
        public boolean trajectoryInProgress = false;
        public double currentTime = 0.0;
        public Timer timer = new Timer();
    }

    public enum ArmMode {
        OPEN_LOOP, MANUAL, TRAJECTORY
    }

    @Override
    public void readPeriodicInputs() {
        io.updateInputs(periodic.inputs);
        visualizer.update(VecBuilder.fill(periodic.inputs.shoulderAbsoluteRad, periodic.inputs.extensionLengthMeters, periodic.inputs.wristAbsoluteRad));

        periodic.openLoopShoulderJoyVal = HIDHelper.getAxisMapped(Constants.Joysticks.SECOND.getRawAxis(2), -0.2, 0.2);
        periodic.openLoopExtensionJoyVal = HIDHelper.getAxisMapped(Constants.Joysticks.SECOND.getRawAxis(1), -0.5, 0.5);
        periodic.openLoopWristJoyVal = HIDHelper.getAxisMapped(Constants.Joysticks.SECOND.getRawAxis(0), -0.5, 0.5);
        // periodic.wristSetpoint = periodic.openLoopWristJoyVal;
        // periodic.extensionSetpoint = periodic.openLoopExtensionJoyVal;
        // periodic.shoulderSetpoint = periodic.openLoopShoulderJoyVal;

        periodic.manualX = HIDHelper.getAxisMapped(Constants.Joysticks.SECOND.getRawAxis(0), -0.3, 0.3);
        periodic.manualY = -HIDHelper.getAxisMapped(Constants.Joysticks.SECOND.getRawAxis(1), 0.5, 1.4);
    }

    public void registerEnabledLoops(ILooper enalbedLooper) {
        enalbedLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {}

            @Override
            public void onLoop(double timestamp) {
                switch (periodic.state) {
                    case OPEN_LOOP:
                        periodic.extensionSetpoint = periodic.openLoopExtensionJoyVal;
                        periodic.shoulderSetpoint = periodic.openLoopShoulderJoyVal;
                        periodic.wristSetpoint = periodic.openLoopWristJoyVal;
                        // double shoulder = shoulderController.calculate(periodic.inputs.shoulderAbsoluteRad, periodic.anglesSetpoint.get(0, 0));
                        // double extension = extensionController.calculate(periodic.inputs.extensionLengthMeters, periodic.anglesSetpoint.get(1, 0));
                        // periodic.extensionSetpoint = Math.signum(extension) * Math.min(0.5, Math.abs(extension));
                        // SmartDashboard.putNumber("Arm/pid", Math.signum(shoulder) * Math.min(0.5, Math.abs(shoulder)));
                    break;
                    case MANUAL:
                    periodic.anglesSetpoint = ArmKinematics.inverseSafe(new Pose2d(periodic.manualX + 1.022, periodic.manualY + 0.7, new Rotation2d()));
                    setpointViz.update(periodic.anglesSetpoint);
                    shoulderController.setTolerance(0.02);
                    extensionController.setTolerance(0.005);
                    wristController.setTolerance(0.02);
                    double shoulder = shoulderController.calculate(periodic.inputs.shoulderAbsoluteRad, periodic.anglesSetpoint.get(0, 0));
                    double extension = extensionController.calculate(periodic.inputs.extensionLengthMeters, periodic.anglesSetpoint.get(1, 0));
                    double wrist = wristController.calculate(periodic.inputs.wristAbsoluteRad, periodic.anglesSetpoint.get(2,0));
                    periodic.shoulderSetpoint = Math.signum(shoulder) * Math.min(0.5, Math.abs(shoulder));
                    periodic.extensionSetpoint = Math.signum(extension) * Math.min(0.5, Math.abs(extension));
                    periodic.wristSetpoint = Math.signum(wrist) * Math.min(0.5, Math.abs(wrist));
                    break;
                    case TRAJECTORY:
                    if(periodic.trajectoryComplete) {
                        setpointViz.update(periodic.holdPose.getAngles());
                        periodic.shoulderSetpoint = shoulderController.calculate(periodic.inputs.shoulderAbsoluteRad, periodic.holdPose.getAngles().get(0, 0));
                        periodic.extensionSetpoint = extensionController.calculate(periodic.inputs.extensionLengthMeters, periodic.holdPose.getAngles().get(1, 0));
                        periodic.wristSetpoint = wristController.calculate(periodic.inputs.wristAbsoluteRad, periodic.holdPose.getAngles().get(2,0));
                    } else if (periodic.trajectoryInProgress && periodic.currentTime == 0) {
                        periodic.timer.restart();
                        periodic.currentTime = periodic.timer.get();
                    } else if (periodic.trajectoryInProgress) {
                        periodic.currentTime = periodic.timer.get();
                        Vector<N3> pose = periodic.currentTrajectory.sample(periodic.currentTime);
                        setpointViz.update(pose);
                        periodic.shoulderSetpoint = shoulderController.calculate(periodic.inputs.shoulderAbsoluteRad, pose.get(0, 0));
                        periodic.extensionSetpoint = extensionController.calculate(periodic.inputs.extensionLengthMeters, pose.get(1, 0));
                        periodic.wristSetpoint = wristController.calculate(periodic.inputs.wristAbsoluteRad, pose.get(2,0));
                        if(periodic.currentTime >= periodic.currentTrajectory.getTotalTime()) {
                            periodic.holdPose = getPresetFromAngles(pose);
                            periodic.trajectoryInProgress = false;
                            periodic.timer.stop();
                            periodic.currentTime = 0.0;
                            periodic.trajectoryComplete = true;
                        }
                    }
                    break;
                }
            }
            @Override
            public void onStop(double timestamp) {
                periodic.shoulderSetpoint = 0;
                periodic.extensionSetpoint = 0;
                periodic.wristSetpoint = 0;
            }
        });
    }
    @Override
    public void writePeriodicOutputs() {
        io.setExtensionPercent(periodic.extensionSetpoint);
        io.setShoulderPercent(periodic.shoulderSetpoint);
        io.setWristPercent(periodic.wristSetpoint);
        io.setIntakePercent(0);
    }
    
    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Arm/currentTime", periodic.currentTime);
        SmartDashboard.putBoolean("Arm/Currently Following Traj", periodic.trajectoryInProgress);

        SmartDashboard.putNumber("Arm/Shoulder/Position", periodic.inputs.shoulderBuiltinTicks);
        SmartDashboard.putNumber("Arm/Shoulder/Abs Position", periodic.inputs.shoulderAbsoluteRad);
        SmartDashboard.putNumber("Arm/Shoulder/Percent Setpoint", periodic.shoulderSetpoint);
        SmartDashboard.putNumber("Arm/Shoulder/Actual Amps", periodic.inputs.shoulderAppliedPower);
        SmartDashboard.putNumberArray("Arm/Shoulder/Manual Pose Setpoint", new double[] {
            periodic.anglesSetpoint.get(0, 0), periodic.anglesSetpoint.get(1, 0), periodic.anglesSetpoint.get(2, 0)
        });
        SmartDashboard.putNumber("Arm/Shoulder/Err", shoulderController.getPositionError());

        SmartDashboard.putNumber("Arm/Extension/Position", periodic.inputs.extensionBuiltinTicks);
        SmartDashboard.putNumber("Arm/Extension/Percent Setpoint", periodic.extensionSetpoint);
        SmartDashboard.putNumber("Arm/Extension/Actual Amps", periodic.inputs.extensionAppliedPower);
        SmartDashboard.putNumber("Arm/Extension/Meters", periodic.inputs.extensionLengthMeters);
        SmartDashboard.putNumber("Arm/Extension/Err", extensionController.getPositionError());

        SmartDashboard.putNumber("Arm/Wrist/Position", periodic.inputs.wristBuiltinTicks);
        SmartDashboard.putNumber("Arm/Wrist/Percent Setpoint", periodic.wristSetpoint);
        SmartDashboard.putNumber("Arm/Wrist/Actual Amps", periodic.inputs.wristAppliedPower);
        SmartDashboard.putNumber("Arm/Wrist/Abs Encoder", periodic.inputs.wristAbsoluteRad);
        SmartDashboard.putNumber("Arm/Wrist/Err", wristController.getPositionError());
    }

    @Override
    public void reset() {}

    public void setMode(ArmMode mode) {
        periodic.state = mode;
    }

    public void setFollowingTrajectory(boolean isInProgress, boolean isComplete) {
        periodic.trajectoryInProgress = isInProgress;
        periodic.trajectoryComplete = isComplete;
        periodic.currentTrajectory = trajectories.getTrajectory(6);
    }

    public void setNewTrajectoryAndFollow(ArmPose.Preset desiredPose) {
        periodic.trajectoryInProgress = true;
        periodic.trajectoryComplete = false;
        periodic.currentTrajectory = trajectories.getTrajectory(periodic.holdPose.getAngles(), desiredPose.getAngles());
    }
    
    public ArmPose.Preset getCurrentPose() {
        return periodic.holdPose;
    }

    public ArmPose.Preset getPresetFromAngles(Vector<N3> angles) {
        ArmPose.Preset returnPreset = ArmPose.Preset.HIGH;
        for (ArmPose.Preset preset : ArmPose.Preset.values()) {
            if(preset.getAngles().isEqual(angles, 0.05)) {
                returnPreset = preset;
            }
        }
        return returnPreset;
    }
}
