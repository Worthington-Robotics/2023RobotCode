package frc.robot.actions.manipulator;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Manipulator;

public class RunPivotAction extends Action{

    @Override
    public void onStart() {
        Manipulator.getInstance().increasePivotPower();   
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
        Manipulator.getInstance().decreasePivotPower();  
    }
    
}
