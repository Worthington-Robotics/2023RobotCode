package frc.robot.actions.wait;

import frc.robot.subsystems.DriveTrain;
import frc.lib.statemachine.Action;

public class ReachLineWaitAction extends Action{

    boolean isFwd;
    double line;

    public ReachLineWaitAction(boolean isFwd, double line) {
        this.isFwd = isFwd;
        this.line = line;
    }

    @Override
    public void onStart() {
    }

    @Override
    public void onLoop() {
    }

    @Override
    public boolean isFinished() {
        return (DriveTrain.getInstance().getEncoderTicks() < line && !isFwd)
        || (DriveTrain.getInstance().getEncoderTicks() > line && isFwd);
    }

    @Override
    public void onStop() {
    }
    
}
