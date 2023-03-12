package frc.robot.actions.drive;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.DriveTrain;

public class UnblockingMoveAction extends Action {
    double targetDistance;
    double desiredHeading;

    public UnblockingMoveAction (double targetDistance, double desiredHeading){
        this.targetDistance = targetDistance;
        this.desiredHeading = desiredHeading;
    }

    @Override
    public void onStart() {
        DriveTrain.getInstance().setDesiredHeading(desiredHeading);
        DriveTrain.getInstance().setMoveForward(targetDistance);
    }

    @Override
    public void onLoop() {}

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void onStop() {
    }
}