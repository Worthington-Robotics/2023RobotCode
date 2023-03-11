package frc.robot.actions.wait;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Manipulator;
import frc.lib.statemachine.Action;
import frc.lib.util.TimerBoolean;
import frc.robot.Constants;

public class PoseWaitAction extends Action{

    TimerBoolean acc = new TimerBoolean(.3);
    @Override
    public void onStart() {
    }

    @Override
    public void onLoop() {
        if(Arm.getInstance().getAccepted()) {
            acc.start();
        } else {
            acc.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return acc.getBoolean();
    }

    @Override
    public void onStop() {
    }
    
}
