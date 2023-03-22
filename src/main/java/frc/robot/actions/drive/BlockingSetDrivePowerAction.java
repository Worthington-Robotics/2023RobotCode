package frc.robot.actions.drive;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.DriveTrain;

public class SetDrivePowerAction extends Action{
    double desiredHeading;
    double motorPower;

	public SetDrivePowerAction(double desiredHeading, double motorPower) {
		this.desiredHeading = desiredHeading;
        this.motorPower = motorPower;
	}

	@Override
	public void onStart() {
		DriveTrain.getInstance().setDesiredHeading(desiredHeading);
		DriveTrain.getInstance().setMotorPower(motorPower);
	}

	@Override
	public void onLoop() {}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void onStop() {
        DriveTrain.getInstance().setStopped();
    }
}
