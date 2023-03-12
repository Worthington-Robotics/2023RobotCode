package frc.robot.actions.wait;

import frc.robot.subsystems.DriveTrain;
import frc.lib.statemachine.Action;
import frc.robot.Constants;

public class DriveLevelWaitAction extends Action{

    @Override
    public void onStart() {
    }

    @Override
    public void onLoop() {
    }

    @Override
    public boolean isFinished() {
        return DriveTrain.getInstance().getDriveLevelAccepted();
    }

    @Override
    public void onStop() {
        DriveTrain.getInstance().resetEncoders();
        DriveTrain.getInstance().setStopped();
    }
    
}