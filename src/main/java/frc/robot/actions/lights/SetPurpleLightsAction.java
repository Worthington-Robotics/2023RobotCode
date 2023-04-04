package frc.robot.actions.lights;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Lights.State;

public class SetPurpleLightsAction extends Action{

    @Override
    public void onStart() {
        Lights.getInstance().setLightState(State.LIGHTS_PURPLE);
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