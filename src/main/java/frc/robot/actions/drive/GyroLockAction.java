package frc.robot.actions.drive;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.DriveTrain;

public class GyroLockAction extends Action {
	@Override
	public void onStart() {
		DriveTrain.getInstance().setDesiredHeading(DriveTrain.getInstance().getNormalizedHeading());
		DriveTrain.getInstance().setGyroLock(true);
	}

	@Override
	public void onLoop() {}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void onStop() {
		DriveTrain.getInstance().setGyroLock(false);
	}
}
