package frc.robot.actions.drive;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.DriveTrain;

public class GearChangeAction extends Action {

    @Override
    public void onStart() {
        DriveTrain.getInstance().setHighGear();
    }

    @Override
    public void onLoop() {

    }

    @Override
    public void onStop() {
        DriveTrain.getInstance().setLowGear();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
