package frc.robot.actions;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.DriveTrain;

public class MoveForward extends Action{
    double targetDistance;
    double desiredHeading;

    public MoveForward (double targetDistance, double desiredHeading){
        this.targetDistance = targetDistance;
        this.desiredHeading = desiredHeading;
    }

    @Override
    public void onStart() {
        DriveTrain.getInstance().setDesiredHeading(this.desiredHeading);
        DriveTrain.getInstance().setTargetDistance(this.targetDistance);
        DriveTrain.getInstance().setMoveForward();
        
    }

    @Override
    public void onLoop() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(DriveTrain.getInstance().getEncoderError()) < 5000.0) { //this cheks the exit condition
            return true;
        }
        return false;
    }

    @Override
    public void onStop() {
        DriveTrain.getInstance().resetEncoders();
        
    }
    
}
