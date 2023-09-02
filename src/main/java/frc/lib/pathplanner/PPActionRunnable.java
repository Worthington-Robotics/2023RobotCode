package frc.lib.pathplanner;

import frc.lib.statemachine.Action;

public class PPActionRunnable extends Thread {
    private Action action;

    public PPActionRunnable(Action action) {
        this.action = action;
    }

    @Override
    public void run() {
        action.onStart();
        while(!action.isFinished()) {
            action.onLoop();
        }
        action.onStop();
    }

    @Override
    public void interrupt() {
        action.onStop();
    }
}
