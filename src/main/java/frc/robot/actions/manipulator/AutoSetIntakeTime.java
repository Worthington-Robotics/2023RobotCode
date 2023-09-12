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
    public void onLoop() {}

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void onStop() {}
    
}