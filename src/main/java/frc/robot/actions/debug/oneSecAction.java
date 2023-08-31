package frc.robot.actions.debug;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.statemachine.Action;

public class oneSecAction extends Action {
    private Timer timer = new Timer();
    @Override
    public void onStart() {
        timer.reset();
        timer.start();
        System.out.println("Started 1 second action");
    }

    @Override
    public void onLoop() {}

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(1.00);
    }

    @Override
    public void onStop() {
        System.out.print("Stopped 1 second timer");
    }
    
}
