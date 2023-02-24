package frc.robot.actions.superstructure;

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
        SuperStructure.getInstance().setButtonPressed();
    }

    @Override
    public void onLoop() {}

    @Override
    public void onStop() {
        SuperStructure.getInstance().setIntakePower(0);
        SuperStructure.getInstance().setButtonPressed();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}