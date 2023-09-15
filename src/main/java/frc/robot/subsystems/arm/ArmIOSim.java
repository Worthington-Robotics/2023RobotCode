package frc.robot.subsystems.arm;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class ArmIOSim implements ArmIO {
    private SingleJointedArmSim armSim;

    public ArmIOSim() {
        armSim = new SingleJointedArmSim(DCMotor.getFalcon500(1), Constants.Arm.PHYSICS_SHOULDER_GEAR_RATIO, Constants.Arm.PHYSICS_TEST_KgPerMetersSquared, 1, -1.35, 1.35, false);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        armSim.update(0.02);
        inputs.shoulderAppliedPower = armSim.getCurrentDrawAmps();
        inputs.shoulderAbsoluteRad = armSim.getAngleRads();

        inputs.extensionLengthMeters = 1;
    }

    @Override
    public void setShoulderPercent(double percent) {
        armSim.setInputVoltage(12 * percent);
    }
}
