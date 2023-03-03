package frc.robot.actions.superstructure;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.SuperStructure;

public class SwitchSolenoid extends Action {
	public SwitchSolenoid() {}

	@Override
	public void onStart() {
		SuperStructure.getInstance().setIntakeUp();
	}

	@Override
	public void onLoop() {}

	@Override
	public void onStop() {
		SuperStructure.getInstance().setIntakeDown();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
