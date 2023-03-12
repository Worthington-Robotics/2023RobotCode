package frc.robot.actions.drive;

import frc.lib.control.ErrorChecker;
import frc.lib.statemachine.Action;
import frc.robot.subsystems.DriveTrain;
import frc.robot.Constants;

public class MoveForwardAction extends Action {
    double targetDistance;
    double desiredHeading;

    public MoveForwardAction (double targetDistance, double desiredHeading){
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
    }

    @Override
    public void onStop() {
    }
}
