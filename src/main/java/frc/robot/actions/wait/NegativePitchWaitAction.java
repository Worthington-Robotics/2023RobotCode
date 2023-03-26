package frc.robot.actions.wait;

import frc.robot.subsystems.DriveTrain;
import frc.lib.statemachine.Action;

public class NegativePitchWaitAction extends Action{
    double pitch;

    @Override
    public void onStart() {
        pitch = DriveTrain.getInstance().getLevelError();
    }

    @Override
    public void onLoop() {
        pitch = DriveTrain.getInstance().getLevelError();
    }

    @Override
    public boolean isFinished() {
       return pitch <= -7;
    }

    @Override
    public void onStop() {
    }
    
}

