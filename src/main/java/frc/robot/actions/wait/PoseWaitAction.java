package frc.robot.actions.wait;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Arm;

public class PoseWaitAction extends Action{
    @Override
    public void onStart() {}

    @Override
    public void onLoop() {}

    @Override
    public boolean isFinished() {
        return Arm.getInstance().getAccepted();
    }

    @Override
    public void onStop() {}
    
}