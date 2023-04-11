package frc.robot.actions.wait;

import frc.robot.subsystems.DriveTrain;
import frc.lib.statemachine.Action;

public class LevelPitchWaitAction extends Action{
    double pitch;
    double pitchMax;

    public LevelPitchWaitAction(double pitchMax){
        this.pitchMax = pitchMax;
    }

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
       return Math.abs(pitch) <= Math.abs(pitchMax);
    }

    @Override
    public void onStop() {
    }
    
}