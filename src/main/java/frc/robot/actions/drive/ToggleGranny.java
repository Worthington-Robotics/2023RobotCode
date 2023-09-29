package frc.robot.actions.drive;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.SwerveDrive;

public class ToggleGranny extends Action {

    @Override
    public void onStart() {
        SwerveDrive.getInstance().setGrannyMode(false);
    }

    @Override
    public void onLoop() {}

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void onStop() {
        SwerveDrive.getInstance().setGrannyMode(true);
    }
    
}
