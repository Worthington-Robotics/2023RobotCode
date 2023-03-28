package frc.robot.actions.wait;

import frc.robot.subsystems.DriveTrain;
import frc.lib.statemachine.Action;

public class LevelPitchWaitAction extends Action{
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
       return Math.abs(pitch) <= 3.0;
    }

    @Override
    public void onStop() {
    }
    
}

