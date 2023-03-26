package frc.robot.actions.wait;

import frc.robot.subsystems.DriveTrain;
import frc.lib.statemachine.Action;

public class DeltaPitchWaitAction extends Action{
    boolean moveForward;

    public DeltaPitchWaitAction(boolean forward) {
        this.moveForward = forward;
    }

    @Override
    public void onStart() {
    }

    @Override
    public void onLoop() {
    }

    @Override
    public boolean isFinished() {
        return DriveTrain.getInstance().getDeltaPitchAccepted(this.moveForward);
    }

    @Override
    public void onStop() {
        DriveTrain.getInstance().resetEncoders();
        DriveTrain.getInstance().setStopped();
    }
    
}