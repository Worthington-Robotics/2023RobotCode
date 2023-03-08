package frc.robot.actions.wait;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.DriveTrain;

public class LineCrossWaitAction extends Action {
	private double distance;
	public LineCrossWaitAction(double distance) {
		this.distance = distance;
	}

	@Override
	public void onStart() {}

	@Override
	public void onLoop() {}

	@Override
	public void onStop() {}

	@Override
	public boolean isFinished() {
		final double average = (DriveTrain.getInstance().getLeftEncoderDistance()
			+ DriveTrain.getInstance().getRightEncoderDistance()) / 2.0;
		return average >= distance;
	}
}