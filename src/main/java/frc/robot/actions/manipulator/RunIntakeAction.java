package frc.robot.actions.manipulator;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Manipulator;

public class RunIntakeAction extends Action {
    // The speed to run the intake at
    double power;
    
    public RunIntakeAction(double power) {
        this.power = power;
    }

    @Override
    public void onStart() {
        Manipulator.getInstance().setIntakePower(power);
    }

    @Override
    public void onLoop() {}

    @Override
    public void onStop() {
        if(power > 0)
            Manipulator.getInstance().setIntakePower(0.1);
        else 
            Manipulator.getInstance().setIntakePower(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}