package frc.robot.subsystems.arm;

public interface ArmIO {
    public class ArmIOInputs {
        public double shoulderBuiltinTicks = 0;
        public double shoulderAbsoluteRad = 0;
        public double shoulderAppliedPower = 0;

        public double extensionBuiltinTicks = 0;
        public double extensionAbsoluteRad = 0;
        public double extensionAppliedPower = 0;
        public double extensionLengthMeters = 0;

        public double wristBuiltinTicks = 0;
        public double wristAbsoluteRad = 0;
        public double wristAppliedPower = 0;

        public double intakeAppliedPower = 0;
    }

    public default void updateInputs(ArmIOInputs inputs) {}
    public default void setWristPercent(double percent) {}
    public default void setExtensionPercent(double percent) {}
    public default void setShoulderPercent(double percent) {}
    public default void setIntakePercent(double percent) {}
    public default void setBrakeMode(boolean shoulderBrake, boolean extensionBrake, boolean wristBrake) {}
}
