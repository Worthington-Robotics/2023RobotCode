package frc.robot.actions.arm;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Arm;

public class DisableLimitSwitchAction extends Action{

    @Override
    public void onStart() {
        Arm.getInstance().setDisabledLimitSwtich();
    }

    @Override
    public void onLoop() { 
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void onStop() {
        // TODO Auto-generated method stub
        
    }
    
}
