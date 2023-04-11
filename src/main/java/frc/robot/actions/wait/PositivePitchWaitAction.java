package frc.robot.actions.wait;

import frc.robot.subsystems.DriveTrain;
import frc.lib.statemachine.Action;

public class PositivePitchWaitAction extends Action{
    double pitch;
    double pitchMin;

    public PositivePitchWaitAction(double pitchMin){
        this.pitchMin = pitchMin;
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
       return pitch >= pitchMin;
    }

    @Override
    public void onStop() {
    }
    
}
