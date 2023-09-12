package frc.robot.actions.manipulator;

import frc.lib.statemachine.Action;
import frc.robot.Constants;
import frc.robot.subsystems.Manipulator;

public class SpitGamePiece extends Action {
    double power = Constants.Arm.ANYTHING_OUT_POWER;
    
    @Override
    public void onStart() {
        Manipulator.getInstance().setIntakePower(power);
    }

    @Override
    public void onLoop() {}

    @Override
    public boolean isFinished() {
        return !Manipulator.getInstance().isObject();
    }

    @Override
    public void onStop() {
        Manipulator.getInstance().setIntakePower(0.0);
        Manipulator.getInstance().setGamePiece(false);
    }
    
}
