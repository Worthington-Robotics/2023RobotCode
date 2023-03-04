package frc.robot.actions.arm;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Arm;

public class EnableLimitSwitchAction extends Action{

    @Override
    public void onStart() {
        Arm.getInstance().setEnabledLimitSwitch();   
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
    }
    
}
