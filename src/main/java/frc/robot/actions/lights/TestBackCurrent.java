package frc.robot.actions.lights;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Lights.State;

public class TestBackCurrent extends Action {

    @Override
    public void onStart() {
        if (Lights.getInstance().getState() == State.HAS_GAMEPIECE) {
            Lights.getInstance().setGamepiece(false);
        } else {
            Lights.getInstance().setGamepiece(true);
        }
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
