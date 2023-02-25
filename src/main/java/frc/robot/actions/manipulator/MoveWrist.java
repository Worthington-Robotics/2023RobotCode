package frc.robot.actions.manipulator;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Manipulator;

public class MoveWrist extends Action {
	
	double power;
	public MoveWrist(double power) {
		this.power = power;
	}

	@Override
	public void onStart() {
		Manipulator.getInstance().setWristPower(power);
	}

	@Override
	public void onLoop() {}

	@Override
	public void onStop() {
		Manipulator.getInstance().setWristPower(0);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
