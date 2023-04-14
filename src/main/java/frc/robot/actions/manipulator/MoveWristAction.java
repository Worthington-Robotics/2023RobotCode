package frc.robot.actions.manipulator;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Manipulator;
public class MoveWristAction extends Action {
	
	double power;
	public MoveWristAction(double power) {
		this.power = power;
	}

	@Override
	public void onStart() {
		if(power > 0) {
			Manipulator.getInstance().incWrist();
		} else {
			Manipulator.getInstance().decWrist();
		}
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
