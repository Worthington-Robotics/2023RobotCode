package frc.robot.actions.arm;
import frc.lib.statemachine.Action;
import frc.robot.subsystems.Arm;

public class PivotMoveAction extends Action {
	
	double power;
	public PivotMoveAction(double power) {
		this.power = power;
	}

	@Override
	public void onStart() {
		Arm.getInstance().setPivotPower(power);
	}

	@Override
	public void onLoop() {}

	@Override
	public void onStop() {
		Arm.getInstance().setPivotPower(0);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}