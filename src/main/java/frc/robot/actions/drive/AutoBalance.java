package frc.robot.actions.drive;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveDrive.State;

public class AutoBalance extends Action {

    @Override
    public void onStart() {
        SwerveDrive.getInstance().setState(State.AutoBalance);
    }

    @Override
    public void onLoop() {
        SwerveDrive.getInstance().setState(State.AutoBalance);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void onStop() {
        SwerveDrive.getInstance().setState(State.AutoBalance);
    }
    
}
