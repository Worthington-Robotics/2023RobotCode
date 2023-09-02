package frc.robot.actions.lights;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Lights;

public class SetCube extends Action {

    @Override
    public void onStart() {
        Lights.getInstance().setCube();
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
