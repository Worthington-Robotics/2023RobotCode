package frc.robot.subsystems.arm;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class ArmIOSim implements ArmIO {
    private SingleJointedArmSim shoulderSim;
    private ElevatorSim extensionSim;
    private SingleJointedArmSim wristSim;

    public ArmIOSim() {
        shoulderSim = new SingleJointedArmSim(DCMotor.getFalcon500(1), Constants.Arm.PHYSICS_SHOULDER_GEAR_RATIO * 2051, Constants.Arm.PHYSICS_TEST_KgPerMetersSquared, 0.9, -1.35, 1.35, true);
        extensionSim = new ElevatorSim(DCMotor.getFalcon500(1), 3, 1.5, 0.01, 0.69, 1.37, true);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        shoulderSim.update(0.02);
        extensionSim.update(0.02);
        inputs.shoulderAppliedPower = shoulderSim.getVelocityRadPerSec();
        inputs.shoulderAbsoluteRad = shoulderSim.getAngleRads();

        inputs.extensionLengthMeters = extensionSim.getPositionMeters();
    }

    @Override
    public void setShoulderPercent(double percent) {
        shoulderSim.setInputVoltage(12 * percent);
    }

    @Override
    public void setExtensionPercent(double percent) {
        extensionSim.setInputVoltage(12 * percent);
    }
}
