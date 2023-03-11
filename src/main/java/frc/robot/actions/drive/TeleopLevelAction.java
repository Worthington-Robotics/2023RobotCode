package frc.robot.actions.drive;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.PoseEstimator;

public class TeleopLevelAction extends Action {
	public TeleopLevelAction() {}

	@Override
	public void onStart() {
		DriveTrain.getInstance().setDesiredHeading(DriveTrain.getInstance().getNormalizedHeading());
		DriveTrain.getInstance().setAutoLevel();
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
