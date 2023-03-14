package frc.robot.actions.drive;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.DriveTrain;

public class DriveLevelAction extends Action {
	double desiredHeading;
	boolean moveForward;
	public DriveLevelAction(double desiredHeading, boolean moveForward) {
		this.desiredHeading = desiredHeading;
		this.moveForward = moveForward;
	}

	@Override
	public void onStart() {
		DriveTrain.getInstance().setDesiredHeading(desiredHeading);
		DriveTrain.getInstance().setAutoLevel(moveForward);
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
