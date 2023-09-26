package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.Constants;

public class ArmIOFalcon implements ArmIO {
    private final TalonFX shoulderFalcon500;
    // private final CANCoder shoulderCANCoder;

    private final TalonFX extensionFalcon500;

    private final TalonFX wristFalcon500;

    private final TalonFX intakeFalcon500;

    public ArmIOFalcon() {
        // shoulderCANCoder = new CANCoder(Constants.Arm.SHOULDER_CANCODER_ID);
        shoulderFalcon500 = new TalonFX(Constants.Arm.ARM_ARM_M_ID, "Default Name");
        extensionFalcon500 = new TalonFX(Constants.Arm.ARM_EXTENSION_ID, "Default Name");
        wristFalcon500 = new TalonFX(Constants.Arm.WRIST_MOTOR_ID, "Default Name");
        intakeFalcon500 = new TalonFX(Constants.Arm.INTAKE_MOTOR_ID, "Default Name");

        extensionFalcon500.setInverted(true);
        shoulderFalcon500.setInverted(true);

        extensionFalcon500.setNeutralMode(NeutralMode.Brake);
        shoulderFalcon500.setNeutralMode(NeutralMode.Brake);
        wristFalcon500.setNeutralMode(NeutralMode.Brake);
        intakeFalcon500.setNeutralMode(NeutralMode.Brake);

        shoulderFalcon500.configVoltageCompSaturation(10);
        extensionFalcon500.configVoltageCompSaturation(10);
        wristFalcon500.configVoltageCompSaturation(10);
        intakeFalcon500.configVoltageCompSaturation(10);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.extensionAppliedPower = extensionFalcon500.getSelectedSensorVelocity();
        inputs.extensionBuiltinTicks = extensionFalcon500.getSelectedSensorPosition();
        inputs.extensionAppliedPower = extensionFalcon500.getStatorCurrent();
        inputs.extensionLengthMeters = (((extensionFalcon500.getSelectedSensorPosition()) - 15380) / 230186.1) + 0.8; //GET REAL VALUE

        inputs.intakeAppliedPower = intakeFalcon500.getSelectedSensorVelocity();
        inputs.intakeAppliedPower = intakeFalcon500.getStatorCurrent();

        inputs.shoulderBuiltinTicks = shoulderFalcon500.getSelectedSensorPosition();
        inputs.shoulderAppliedPower = shoulderFalcon500.getStatorCurrent();
        inputs.shoulderAppliedPower = shoulderFalcon500.getSelectedSensorVelocity();
        inputs.shoulderAbsoluteRad = ((-(shoulderFalcon500.getSelectedSensorPosition()) / -72165) - 1.33);

        inputs.wristAppliedPower = wristFalcon500.getSelectedSensorVelocity();
        inputs.wristBuiltinTicks = wristFalcon500.getSelectedSensorPosition();
        inputs.wristAppliedPower = wristFalcon500.getStatorCurrent();
        inputs.wristAbsoluteRad = (wristFalcon500.getSelectedSensorPosition() / 29000) + 1.78;
    }

    @Override
    public void setWristPercent(double percent) {
        wristFalcon500.set(ControlMode.PercentOutput, percent);
        // wristFalcon500.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void setExtensionPercent(double percent) {
        extensionFalcon500.set(ControlMode.PercentOutput, percent);
        // extensionFalcon500.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void setIntakePercent(double percent) {
        intakeFalcon500.set(ControlMode.PercentOutput, percent);
        // intakeFalcon500.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void setShoulderPercent(double percent) {
        shoulderFalcon500.set(ControlMode.PercentOutput, percent);
        // shoulderFalcon500.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void setBrakeMode(boolean shoulderBrake, boolean extensionBrake, boolean wristBrake) {
        if(shoulderBrake) {
            shoulderFalcon500.setNeutralMode(NeutralMode.Brake);
        } else {shoulderFalcon500.setNeutralMode(NeutralMode.Coast);}
        if(extensionBrake) {
            extensionFalcon500.setNeutralMode(NeutralMode.Brake);
        } else {extensionFalcon500.setNeutralMode(NeutralMode.Coast);}
        if(wristBrake) {
            wristFalcon500.setNeutralMode(NeutralMode.Brake);
        } else {wristFalcon500.setNeutralMode(NeutralMode.Coast);}
    }
}
