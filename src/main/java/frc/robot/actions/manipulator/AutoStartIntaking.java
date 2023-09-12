package frc.robot.actions.manipulator;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Manipulator;

public class AutoStartIntaking extends Action {

    @Override
    public void onStart() {
        Manipulator.getInstance().setGamePiece(true);
    }

    @Override
    public void onLoop() {}

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void onStop() {}
    
}