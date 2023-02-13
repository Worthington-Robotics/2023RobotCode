package frc.robot.actions.drive;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.DriveTrain;
import frc.robot.Constants;

public class MoveForward extends Action{
    double targetDistance;
    double desiredHeading;

    public MoveForward (double targetDistance, double desiredHeading){
        this.targetDistance = targetDistance;
        this.desiredHeading = desiredHeading;
    }

    @Override
    public void onStart() {
        DriveTrain.getInstance().setDesiredHeading(desiredHeading);
        DriveTrain.getInstance().setTargetDistance(targetDistance);
        DriveTrain.getInstance().setMoveForward();
    }

    @Override
    public void onLoop() {}

    @Override
    public boolean isFinished() {
        // if (Math.abs(DriveTrain.getInstance().getEncoderError()) < Constants.DRIVE_FORWARD_ACCEPTED_ERROR) { //this cheks the exit condition
        //     return true;
        // }
        return false;
    }

    @Override
    public void onStop() {
        DriveTrain.getInstance().resetEncoders();
        
    }
    
}
