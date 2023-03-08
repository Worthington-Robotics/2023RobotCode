package frc.robot.actions.wait;

import frc.lib.statemachine.Action;

public class DoNothingAction extends Action {
	public DoNothingAction() {}

	@Override
	public void onStart() {}

	@Override
	public void onLoop() {}

	@Override
	public void onStop() {}

	@Override
	public boolean isFinished() {
		return false;
	}
}