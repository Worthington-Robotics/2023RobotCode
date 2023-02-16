package frc.robot.actions.drive;

import frc.lib.control.ErrorChecker;
import frc.lib.statemachine.Action;
import frc.robot.subsystems.DriveTrain;
import frc.robot.Constants;

public class MoveForwardAction extends Action {
    double targetDistance;
    double desiredHeading;
    ErrorChecker checker = new ErrorChecker(
        Constants.DRIVE_FORWARD_ACCEPTED_ERROR, Constants.DRIVE_FORWARD_MINIMUM_TIME);

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
        return checker.check(DriveTrain.getInstance().getEncoderError());
    }

    @Override
    public void onStop() {
        DriveTrain.getInstance().resetEncoders();
        DriveTrain.getInstance().setStopped();
    }
}
