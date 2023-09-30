package frc.robot.actions.manipulator;

import frc.lib.statemachine.Action;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Manipulator;

public class IntakeGamePiece extends Action {
    double power = Constants.Arm.INTAKE_POWER;
    
    @Override
    public void onStart() {
        Manipulator.getInstance().setIntakePower(power);
    }

    @Override
    public void onLoop() {}

    @Override
    public boolean isFinished() {
        return Manipulator.getInstance().isObject();
    }

    @Override
    public void onStop() {
        Manipulator.getInstance().setIntakePower(0.1);
        Manipulator.getInstance().setGamePiece(true);
    }
    
}
