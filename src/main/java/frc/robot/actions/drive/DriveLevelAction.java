package frc.robot.actions.drive;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.*;

public class DriveLevelAction extends Action {
	double desiredHeading;
	double error;
	public DriveLevelAction(double desiredHeading) {
		this.desiredHeading = desiredHeading;
	}

	@Override
	public void onStart() {
		DriveTrain.getInstance().setAutoLevel();
	}

	@Override
	public void onLoop() {
		error = DriveTrain.getInstance().getLevelError();
	}

	@Override
	public boolean isFinished() {
		//if(Math.abs(error) < Constants.)
		return false;
	}

	@Override
	public void onStop() {}
}
