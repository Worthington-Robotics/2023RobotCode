package frc.robot.actions.drive;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.DriveTrain;

public class DriveResetGyroAction extends Action {
	double heading;
	public DriveResetGyroAction(double heading) {
		this.heading = heading;
	}

	@Override
	public void onStart() {
		DriveTrain.getInstance().setGyro(0);
	}

	@Override
	public void onLoop() {}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void onStop() {
	}
}
