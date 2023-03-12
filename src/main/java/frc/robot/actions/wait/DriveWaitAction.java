package frc.robot.actions.wait;

import frc.robot.subsystems.DriveTrain;
import frc.lib.statemachine.Action;
import frc.robot.Constants;

public class DriveWaitAction extends Action{

    boolean crossedLine = false;

    @Override
    public void onStart() {
    }

    @Override
    public void onLoop() {
        if(DriveTrain.getInstance().getEncoderTicks() < DriveTrain.getInstance().getTargetDistance()){
            crossedLine = true;
        } else {
            crossedLine = false;
        }
    }

    @Override
    public boolean isFinished() {
        return crossedLine;
    }

    @Override
    public void onStop() {
        DriveTrain.getInstance().setStopped();
        DriveTrain.getInstance().resetEncoders();
    }
    
}
