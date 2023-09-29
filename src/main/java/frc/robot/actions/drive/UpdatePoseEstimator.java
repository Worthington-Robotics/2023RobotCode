package frc.robot.actions.drive;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.SwerveDrive;

public class UpdatePoseEstimator extends Action {

    @Override
    public void onStart() {
        SwerveDrive.getInstance().setVisionUpdates(true);
    }

    @Override
    public void onLoop() {}

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void onStop() {
        SwerveDrive.getInstance().setVisionUpdates(false);
    }
    
}
