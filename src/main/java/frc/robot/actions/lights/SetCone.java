package frc.robot.actions.lights;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Lights;

public class SetCone extends Action {

    @Override
    public void onStart() {
        Lights.getInstance().setCone();
    }

    @Override
    public void onLoop() {
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void onStop() {
    }
    
}
