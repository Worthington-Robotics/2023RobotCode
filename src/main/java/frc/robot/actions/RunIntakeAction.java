package frc.robot.actions;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.SuperStructure;

public class RunIntakeAction extends Action {
    // The speed to run the intake at
    double power;
    
    public RunIntakeAction(double power) {
        this.power = power;
    }

    @Override
    public void onStart() {
        SuperStructure.getInstance().setIntakePower(power);
    }

    @Override
    public void onLoop() {}

    @Override
    public void onStop() {
        SuperStructure.getInstance().setIntakePower(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}