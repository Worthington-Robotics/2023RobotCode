package frc.robot.subsystems.arm;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.loops.ILooper;
import frc.lib.loops.Loop;
import frc.lib.util.HIDHelper;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Subsystem;
import frc.robot.subsystems.arm.ArmIO.ArmIOInputs;
import frc.robot.subsystems.arm.ArmTrajectory.Parameters;

public class Arm extends Subsystem {
    private static Arm instance = new Arm();
    private ArmNewIO periodic = new ArmNewIO();
    public static Arm getInstance() {return instance;}

    private ArmIO io;
    private TrapezoidProfile shoulderTrapezoid;
	private ArmVisualizer visualizer;
    private ArmVisualizer setpointViz;

    private PIDController shoulderController = new PIDController(12, 0, 0.5);
    private PIDController extensionController = new PIDController(1.0, 0, 0.5);
    private PIDController wristController = new PIDController(0.5, 0, 0.005);

    public Arm() {
        if(Robot.isSimulation()) {
            io = new ArmIOSim();
        } else {io = new ArmIOFalcon();}
        visualizer = new ArmVisualizer(Constants.Arm.ZERO_ANGLES, "Actual");
        setpointViz = new ArmVisualizer(Constants.Arm.ZERO_ANGLES, "Setpoint");
        shoulderTrapezoid = new TrapezoidProfile(new Constraints(2.0, 0.5), new State(1.0, 0.0));
        periodic.timer.start();
    }

    public class ArmNewIO extends PeriodicIO {
        private ArmTrajectory testTrajectory = new ArmTrajectory(new Parameters(Constants.Arm.ZERO_ANGLES, ArmPose.Preset.MID_CONE.getAngles()));
        public ArmNewIO() {
            List<Vector<N3>> points = new ArrayList<Vector<N3>>();
            points.add(ArmKinematics.inverseSafe(new Pose2d(0.6, -0.6, new Rotation2d())));
            testTrajectory.setPoints(1.5, points);
        }
        public ArmMode state = ArmMode.TRAJECTORY;
        public ArmIOInputs inputs = new ArmIOInputs();
        public Vector<N3> anglesSetpoint = VecBuilder.fill(0, 0, 0);

        public double openLoopShoulderJoyVal = 0;
        public double openLoopExtensionJoyVal = 0;
        public double openLoopWristJoyVal = 0;

        public double shoulderSetpoint = 0; //Shoulder motor setpoint in percent. 0.0 being 0% and 1 being 100%
        public double extensionSetpoint = 0; //Shoulder motor setpoint in percent. 0.0 being 0% and 1 being 100%
        public double wristSetpoint = 0; //Shoulder motor setpoint in percent. 0.0 being 0% and 1 being 100%

        public double manualX = 0;
        public double manualY = 0;

        public ArmTrajectory currentTrajectory = testTrajectory;
        public Vector<N3> holdPose = Constants.Arm.ZERO_ANGLES;
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
        periodic.openLoopExtensionJoyVal = HIDHelper.getAxisMapped(Constants.Joysticks.SECOND.getRawAxis(1), -0.1, 0.1);
        periodic.openLoopWristJoyVal = HIDHelper.getAxisMapped(Constants.Joysticks.SECOND.getRawAxis(0), -0.1, 0.1);

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
                    break;
                    case MANUAL:
                    periodic.anglesSetpoint = ArmKinematics.inverseSafe(new Pose2d(periodic.manualX + 1.022, periodic.manualY + 0.7, new Rotation2d()));
                    setpointViz.update(periodic.anglesSetpoint);
                    shoulderController.setTolerance(0.05);
                    extensionController.setTolerance(0.05);
                    wristController.setTolerance(0.05);
                    periodic.shoulderSetpoint = shoulderController.calculate(periodic.inputs.shoulderAbsoluteRad, periodic.anglesSetpoint.get(0, 0));
                    periodic.extensionSetpoint = extensionController.calculate(periodic.inputs.extensionLengthMeters, periodic.anglesSetpoint.get(1, 0));
                    periodic.wristSetpoint = wristController.calculate(periodic.inputs.wristAbsoluteRad, periodic.anglesSetpoint.get(2,0));
                    break;
                    case TRAJECTORY:
                    if(periodic.trajectoryComplete) {
                        setpointViz.update(periodic.holdPose);
                        periodic.shoulderSetpoint = shoulderController.calculate(periodic.inputs.shoulderAbsoluteRad, periodic.holdPose.get(0, 0));
                        periodic.extensionSetpoint = extensionController.calculate(periodic.inputs.extensionLengthMeters, periodic.holdPose.get(1, 0));
                        periodic.wristSetpoint = wristController.calculate(periodic.inputs.wristAbsoluteRad, periodic.holdPose.get(2,0));
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
                            periodic.holdPose = pose;
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
        SmartDashboard.putNumber("Arm/Shoulder/Actual Percent", periodic.inputs.shoulderAppliedPower);
        SmartDashboard.putNumberArray("Arm/Shoulder/Manual Pose Setpoint", new double[] {
            periodic.anglesSetpoint.get(0, 0), periodic.anglesSetpoint.get(1, 0), periodic.anglesSetpoint.get(2, 0)
        });

        SmartDashboard.putNumber("Arm/Extension/Position", periodic.inputs.extensionBuiltinTicks);
        SmartDashboard.putNumber("Arm/Extension/Percent Setpoint", periodic.extensionSetpoint);
        SmartDashboard.putNumber("Arm/Extension/Actual Percent", periodic.inputs.extensionAppliedPower);
        SmartDashboard.putNumber("Arm/Extension/Meters", periodic.inputs.extensionLengthMeters);

        SmartDashboard.putNumber("Arm/Wrist/Position", periodic.inputs.wristBuiltinTicks);
        SmartDashboard.putNumber("Arm/Wrist/Percent Setpoint", periodic.wristSetpoint);
        SmartDashboard.putNumber("Arm/Wrist/Actual Percent", periodic.inputs.wristAppliedPower);
        SmartDashboard.putNumber("Arm/Wrist/Abs Encoder", periodic.inputs.wristAbsoluteRad);
    }

    @Override
    public void reset() {}

    public void setMode(ArmMode mode) {
        periodic.state = mode;
    }

    public void setFollowingTrajectory(boolean isInProgress, boolean isComplete) {
        periodic.trajectoryInProgress = isInProgress;
        periodic.trajectoryComplete = isComplete;
    }
    
}
