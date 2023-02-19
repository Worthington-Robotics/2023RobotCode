package frc.robot.actions.arm;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;

public class OpenClaw extends Action {
	public OpenClaw() {}

	@Override
	public void onStart() {
		Arm.getInstance().setClawClosed();
	}

	@Override
	public void onLoop() {}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void onStop() {
		Arm.getInstance().setClawOpen();
	}
}
