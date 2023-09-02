package frc.robot.actions.debug;

import frc.lib.statemachine.Action;

public class logAction extends Action {
    @Override
    public void onStart() {
        System.out.println("Starting log action");
    }

    @Override
    public void onLoop() {
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void onStop() {
        System.out.println("Stopping log action");
    }
    
}
