package frc.robot.actions.superstructure;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.SuperStructure;

public class MoveWrist extends Action {
	
	double power;
	public MoveWrist(double power) {
		this.power = power;
	}

	@Override
	public void onStart() {
		SuperStructure.getInstance().setWristPower(power);
	}

	@Override
	public void onLoop() {}

	@Override
	public void onStop() {
		SuperStructure.getInstance().setWristPower(0);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
