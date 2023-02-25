package frc.robot.actions.manipulator;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Manipulator;

public class RunPivotAction extends Action{

    double power;

    public RunPivotAction (double power){
        this.power = power;
    }

    @Override
    public void onStart() {
        Manipulator.getInstance().setWristPower(power);   
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
        Manipulator.getInstance().setWristPower(0);    
    }
    
}
