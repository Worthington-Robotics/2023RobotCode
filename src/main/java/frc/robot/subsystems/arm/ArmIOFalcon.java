package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import frc.robot.Constants;

public class ArmIOFalcon implements ArmIO {
    private final TalonFX shoulderFalcon500;
    private final CANCoder shoulderCANCoder;

    private final TalonFX extensionFalcon500;

    private final TalonFX wristFalcon500;

    private final TalonFX intakeFalcon500;

    public ArmIOFalcon() {
        shoulderCANCoder = new CANCoder(Constants.Arm.SHOULDER_CANCODER_ID);
        shoulderFalcon500 = new TalonFX(Constants.Arm.ARM_ARM_M_ID);
        extensionFalcon500 = new TalonFX(Constants.Arm.ARM_EXTENSION_ID);
        wristFalcon500 = new TalonFX(Constants.Arm.WRIST_MOTOR_ID);
        intakeFalcon500 = new TalonFX(Constants.Arm.INTAKE_MOTOR_ID);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.extensionAppliedPower = extensionFalcon500.getSelectedSensorVelocity();
        inputs.extensionBuiltinTicks = extensionFalcon500.getSelectedSensorPosition();
        inputs.shoulderAbsoluteRad = shoulderCANCoder.getAbsolutePosition();

        inputs.intakeAppliedPower = intakeFalcon500.getSelectedSensorVelocity();

        inputs.shoulderBuiltinTicks = extensionFalcon500.getSelectedSensorPosition();
        inputs.shoulderAppliedPower = extensionFalcon500.getSelectedSensorVelocity();

        inputs.wristAppliedPower = wristFalcon500.getSelectedSensorVelocity();
        inputs.wristBuiltinTicks = wristFalcon500.getSelectedSensorPosition();
    }

    @Override
    public void setWristPercent(double percent) {
        wristFalcon500.set(ControlMode.PercentOutput, percent);
    }

    @Override
    public void setExtensionPercent(double percent) {
        extensionFalcon500.set(ControlMode.PercentOutput, percent);
    }

    @Override
    public void setIntakePercent(double percent) {
        intakeFalcon500.set(ControlMode.PercentOutput, percent);
    }

    @Override
    public void setShoulderPercent(double percent) {
        shoulderFalcon500.set(ControlMode.PercentOutput, percent);
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