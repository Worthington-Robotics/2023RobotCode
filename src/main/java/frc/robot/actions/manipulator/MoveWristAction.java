package frc.robot.actions.manipulator;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Arm.ArmMode;

public class MoveWristAction extends Action {
	
	double power;
	public MoveWristAction(double power) {
		this.power = power;
	}

	@Override
	public void onStart() {
		if(Arm.getInstance().getMode() != ArmMode.CLOSED_LOOP)
			Manipulator.getInstance().setWristPower(power);
		else if(power > 0) {
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
