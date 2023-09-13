package frc.robot.subsystems.arm;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.loops.ILooper;
import frc.lib.loops.Loop;
import frc.lib.util.HIDHelper;
import frc.robot.Constants;
import frc.robot.subsystems.Subsystem;
import frc.robot.subsystems.arm.ArmIO.ArmIOInputs;

public class ArmNew extends Subsystem {
    private static ArmNew instance = new ArmNew();
    public static ArmNew getInstance() {return instance;}

    private ArmNewIO periodic = new ArmNewIO();

    private ArmIO io = new ArmIOFalcon();
	private ArmKinematics kinematics = new ArmKinematics();
	Vector<N3> angles = kinematics.inverseSafe(ArmPoseNew.Preset.MID_CONE.getPose2d());
	private ArmVisualizer visualizer = new ArmVisualizer(angles);

    public class ArmNewIO extends PeriodicIO {
        public ArmMode state = ArmMode.OPEN_LOOP;
        public ArmIOInputs inputs = new ArmIOInputs();

        public double openLoopShoulderJoyVal = 0.0;
        public double openLoopExtensionJoyVal = 0.0;
        public double openLoopWristJoyVal = 0.0;

        public double shoulderSetpoint = 0.0; //Shoulder motor setpoint in percent. 0.0 being 0% and 1 being 100%
        public double extensionSetpoint = 0.0; //Shoulder motor setpoint in percent. 0.0 being 0% and 1 being 100%
        public double wristSetpoint = 0.0; //Shoulder motor setpoint in percent. 0.0 being 0% and 1 being 100%
    }

    public enum ArmMode {
        OPEN_LOOP, CLOSED_LOOP
    }

    @Override
    public void readPeriodicInputs() {
        io.updateInputs(periodic.inputs);
        visualizer.update(VecBuilder.fill(periodic.inputs.shoulderAbsoluteRad, periodic.inputs.extensionLengthMeters, periodic.inputs.wristAbsoluteRad));

        // periodic.openLoopShoulderJoyVal = HIDHelper.getAxisMapped(Constants.Joysticks.SECOND.getRawAxis(2), -0.1, 0.1);
        periodic.openLoopShoulderJoyVal = Constants.Joysticks.SECOND.getRawAxis(2);
        periodic.openLoopExtensionJoyVal = HIDHelper.getAxisMapped(Constants.Joysticks.SECOND.getRawAxis(1), -0.1, 0.1);
        periodic.openLoopWristJoyVal = HIDHelper.getAxisMapped(Constants.Joysticks.SECOND.getRawAxis(0), -0.1, 0.1);
    }

    public void registerEnabledLoops(ILooper enalbedLooper) {
        enalbedLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {}

            @Override
            public void onLoop(double timestamp) {
                switch (periodic.state) {
                    case OPEN_LOOP:
                        periodic.openLoopExtensionJoyVal = periodic.extensionSetpoint;
                        periodic.openLoopShoulderJoyVal = periodic.shoulderSetpoint;
                        periodic.openLoopWristJoyVal = periodic.wristSetpoint;
                    break;
                    case CLOSED_LOOP:
                        double x = Math.sin(timestamp);
                        visualizer.update(kinematics.inverseSafe(new Pose2d(0.9+(0.3*x), 0.5 *x, new Rotation2d(0))));

                        periodic.shoulderSetpoint = 0;
                        periodic.extensionSetpoint = 0;
                        periodic.wristSetpoint = 0;
                    break;
                }
            }
            @Override
            public void onStop(double timestamp) {}
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
        SmartDashboard.putNumber("Arm/Shoulder/Percent Setpoint", periodic.shoulderSetpoint);
        SmartDashboard.putNumber("Arm/Shoulder/Actual Percent", periodic.inputs.shoulderAppliedPower);

        SmartDashboard.putNumber("Arm/Extension/Position", periodic.inputs.extensionBuiltinTicks);
        SmartDashboard.putNumber("Arm/Extension/Percent Setpoint", periodic.extensionSetpoint);
        SmartDashboard.putNumber("Arm/Extension/Actual Percent", periodic.inputs.extensionAppliedPower);

        SmartDashboard.putNumber("Arm/Wrist/Position", periodic.inputs.wristBuiltinTicks);
        SmartDashboard.putNumber("Arm/Wrist/Percent Setpoint", periodic.wristSetpoint);
        SmartDashboard.putNumber("Arm/Wrist/Actual Percent", periodic.inputs.wristAppliedPower);
    }

    @Override
    public void reset() {}
    
}
