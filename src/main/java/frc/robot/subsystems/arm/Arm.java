package frc.robot.subsystems.arm;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
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
    private ArmNewIO periodic = new ArmNewIO();
    public static Arm getInstance() {return instance;}

    private ArmIO io;
	private ArmKinematics kinematics;
	private ArmVisualizer visualizer;
    private ArmVisualizer setpointViz;

    public Arm() {
        if(Robot.isSimulation()) {
            io = new ArmIOSim();
        } else {io = new ArmIOFalcon();}
        kinematics = new ArmKinematics();
        visualizer = new ArmVisualizer(Constants.Arm.ZERO_ANGLES, "Actual");
        setpointViz = new ArmVisualizer(Constants.Arm.ZERO_ANGLES, "Setpoint");
    }

    public class ArmNewIO extends PeriodicIO {
        public ArmMode state = ArmMode.MANUAL;
        public ArmIOInputs inputs = new ArmIOInputs();

        public double openLoopShoulderJoyVal = 0;
        public double openLoopExtensionJoyVal = 0;
        public double openLoopWristJoyVal = 0;

        public double shoulderSetpoint = 0; //Shoulder motor setpoint in percent. 0.0 being 0% and 1 being 100%
        public double extensionSetpoint = 0; //Shoulder motor setpoint in percent. 0.0 being 0% and 1 being 100%
        public double wristSetpoint = 0; //Shoulder motor setpoint in percent. 0.0 being 0% and 1 being 100%

        public double manualX = 0;
        public double manualY = 0;
    }

    public enum ArmMode {
        OPEN_LOOP, MANUAL, TRAJECTORY
    }

    @Override
    public void readPeriodicInputs() {
        io.updateInputs(periodic.inputs);
        visualizer.update(VecBuilder.fill(periodic.inputs.shoulderAbsoluteRad, periodic.inputs.extensionLengthMeters, Constants.Arm.ZERO_ANGLES.get(2, 0)));

        periodic.openLoopShoulderJoyVal = HIDHelper.getAxisMapped(Constants.Joysticks.SECOND.getRawAxis(2), -0.2, 0.2);
        periodic.openLoopExtensionJoyVal = HIDHelper.getAxisMapped(Constants.Joysticks.SECOND.getRawAxis(1), -0.1, 0.1);
        periodic.openLoopWristJoyVal = HIDHelper.getAxisMapped(Constants.Joysticks.SECOND.getRawAxis(0), -0.1, 0.1);

        periodic.manualX = HIDHelper.getAxisMapped(Constants.Joysticks.SECOND.getRawAxis(0), -0.3, 0.3);
        periodic.manualY = HIDHelper.getAxisMapped(Constants.Joysticks.SECOND.getRawAxis(1), -0.5, 0.5);
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
                    Vector<N3> angles = kinematics.inverseSafe(new Pose2d(periodic.manualX + 1.022, periodic.manualY-0.5, new Rotation2d()));
                    setpointViz.update(angles);
                    PIDController shoulderController = new PIDController(1.2, 0.0, 0.5);
                    PIDController extensionController = new PIDController(1.0, 0, 0);
                    shoulderController.setTolerance(0.05);
                    extensionController.setTolerance(0.05);
                    periodic.shoulderSetpoint = shoulderController.calculate(periodic.inputs.shoulderAbsoluteRad, angles.get(0, 0));
                    periodic.extensionSetpoint = extensionController.calculate(periodic.inputs.extensionLengthMeters, angles.get(1, 0));
                    break;
                    case TRAJECTORY:
                    double x = Math.sin(timestamp);
                    visualizer.update(kinematics.inverseSafe(new Pose2d(0.9+(0.3*x), 0.5 *x, new Rotation2d(0))));

                    periodic.shoulderSetpoint = 0;
                    periodic.extensionSetpoint = 0;
                    periodic.wristSetpoint = 0;
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
        SmartDashboard.putNumber("Arm/Shoulder/Position", periodic.inputs.shoulderBuiltinTicks);
        SmartDashboard.putNumber("Arm/Shoulder/Abs Position", periodic.inputs.shoulderAbsoluteRad);
        SmartDashboard.putNumber("Arm/Shoulder/Percent Setpoint", periodic.shoulderSetpoint);
        SmartDashboard.putNumber("Arm/Shoulder/Actual Percent", periodic.inputs.shoulderAppliedPower);
        SmartDashboard.putNumberArray("Arm/Shoulder/Manual Pose Setpoint", new double[] {
            periodic.manualX, periodic.manualY, 0
        });

        SmartDashboard.putNumber("Arm/Extension/Position", periodic.inputs.extensionBuiltinTicks);
        SmartDashboard.putNumber("Arm/Extension/Percent Setpoint", periodic.extensionSetpoint);
        SmartDashboard.putNumber("Arm/Extension/Actual Percent", periodic.inputs.extensionAppliedPower);
        SmartDashboard.putNumber("Arm/Extension/Meters", periodic.inputs.extensionLengthMeters);

        SmartDashboard.putNumber("Arm/Wrist/Position", periodic.inputs.wristBuiltinTicks);
        SmartDashboard.putNumber("Arm/Wrist/Percent Setpoint", periodic.wristSetpoint);
        SmartDashboard.putNumber("Arm/Wrist/Actual Percent", periodic.inputs.wristAppliedPower);
    }

    @Override
    public void reset() {}

    public void setMode(ArmMode mode) {
        periodic.state = mode;
    }
    
}
