package frc.robot.actions.wait;

import frc.robot.subsystems.Arm;
import frc.lib.statemachine.Action;

public class PoseWaitAction extends Action{
    @Override
    public void onStart() {
    }

    @Override
    public void onLoop() {
    }

    @Override
    public boolean isFinished() {
        return Arm.getInstance().getAccepted();
    }

    @Override
    public void onStop() {
    }
    
}
