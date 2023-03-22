package frc.robot.actions.wait;

import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.statemachine.Action;
import frc.robot.Constants;

public class NegativePitchWaitAction extends Action{
    Timer timer;

    @Override
    public void onStart() {
        timer.start();
    }

    @Override
    public void onLoop() {
        double pitch = DriveTrain.getInstance().getLevelError();
        if(pitch > -3.0){
            timer.reset();
        }
    }

    @Override
    public boolean isFinished() {
       return timer.get() > 0.5;
    }

    @Override
    public void onStop() {
        DriveTrain.getInstance().resetEncoders();
        DriveTrain.getInstance().setStopped();
    }
    
}

