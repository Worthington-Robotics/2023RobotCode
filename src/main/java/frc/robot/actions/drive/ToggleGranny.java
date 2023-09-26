package frc.robot.actions.drive;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.SwerveDrive;

public class ToggleGranny extends Action {

    @Override
    public void onStart() {
        if(SwerveDrive.getInstance().getGrannyMode()) {
            SwerveDrive.getInstance().setGrannyMode(false);
        } else {
            SwerveDrive.getInstance().setGrannyMode(true);
        }
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
