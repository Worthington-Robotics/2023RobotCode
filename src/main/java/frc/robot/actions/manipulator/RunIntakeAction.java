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
        Manipulator.getInstance().setIntakePower(0.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}