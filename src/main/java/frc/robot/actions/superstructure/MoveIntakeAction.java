package frc.robot.actions.superstructure;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.SuperStructure;

public class MoveIntakeAction extends Action {
	SuperStructure.IntakePosition position;
	
	public MoveIntakeAction(SuperStructure.IntakePosition position) {
		this.position = position;
	}

	@Override
	public void onStart() {
		SuperStructure.getInstance().setIntakePosition(position);
	}

	@Override
	public void onLoop() {}

	@Override
	public void onStop() {}

	@Override
	public boolean isFinished() {
		return false;
	}
}
