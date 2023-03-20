package frc.robot.actions.drive;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.DriveTrain;

public class TeleopLevelAction extends Action {
	boolean moveForward;
	public TeleopLevelAction(boolean moveForward) {
		this.moveForward = moveForward;
	}

	@Override
	public void onStart() {
		DriveTrain.getInstance().setDesiredHeading(DriveTrain.getInstance().getNormalizedHeading());
		DriveTrain.getInstance().setAutoLevel(moveForward);
	}

	@Override
	public void onLoop() {}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void onStop() {
		DriveTrain.getInstance().setOpenLoop();
	}
}
