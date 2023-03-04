package frc.robot.actions.drive;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.DriveTrain;

public class DriveLevelAction extends Action {
	double desiredHeading;
	public DriveLevelAction(double desiredHeading) {
		this.desiredHeading = desiredHeading;
	}

	@Override
	public void onStart() {
		DriveTrain.getInstance().setDesiredHeading(desiredHeading);
		DriveTrain.getInstance().setAutoLevel();
	}

	@Override
	public void onLoop() {}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void onStop() {}
}
