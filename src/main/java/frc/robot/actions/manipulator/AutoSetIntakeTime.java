package frc.robot.actions.manipulator;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.statemachine.Action;
import frc.robot.subsystems.Manipulator;

public class AutoSetIntakeTime extends Action {
    public double timeDelay;
    public AutoSetIntakeTime(double delay) {
        this.timeDelay = delay;
    }

    @Override
    public void onStart() {
        //Manipulator.getInstance().setTimoutControlled();
        Manipulator.getInstance().setStartTime(Timer.getFPGATimestamp());
        Manipulator.getInstance().setTimeOffset(timeDelay);
    }

    @Override
    public void onLoop() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return true;
    }

    @Override
    public void onStop() {
        // TODO Auto-generated method stub
        
    }
    
}
