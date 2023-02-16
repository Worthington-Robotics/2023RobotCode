package frc.robot.actions.drive;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.statemachine.Action;
import frc.robot.subsystems.DriveTrain;
import frc.robot.Constants;

public class MoveForwardAction extends Action {
    double targetDistance;
    double desiredHeading;
    double startTime;

    public MoveForwardAction (double targetDistance, double desiredHeading){
        this.targetDistance = targetDistance;
        this.desiredHeading = desiredHeading;
    }

    @Override
    public void onStart() {
        DriveTrain.getInstance().setDesiredHeading(desiredHeading);
        DriveTrain.getInstance().setMoveForward(targetDistance);
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void onLoop() {}

    @Override
    public boolean isFinished() {
        return (Math.abs(DriveTrain.getInstance().getEncoderError()) < Constants.DRIVE_FORWARD_ACCEPTED_ERROR
        && (Timer.getFPGATimestamp() - startTime) > Constants.DRIVE_FORWARD_MINIMUM_TIME);
    }

    @Override
    public void onStop() {
        DriveTrain.getInstance().resetEncoders();
        DriveTrain.getInstance().setStopped();
    }
}
