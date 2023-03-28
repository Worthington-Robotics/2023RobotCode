package frc.robot.actions.drive;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.DriveTrain;

public class ReverseDriveAction extends Action {

    @Override
    public void onStart() {
        if(DriveTrain.getInstance().getReverseDrive()){
            DriveTrain.getInstance().setReverseDriveTrain(false);
        } else {
            DriveTrain.getInstance().setReverseDriveTrain(true);
        }
    }

    @Override
    public void onLoop() {}

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void onStop() {
    }
}
