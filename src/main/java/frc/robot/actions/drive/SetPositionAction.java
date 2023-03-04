package frc.robot.actions.drive;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Rotation2d;
import frc.lib.statemachine.Action;
import frc.robot.subsystems.PoseEstimator;

public class SetPositionAction extends Action {
	double x;
	double y;
	double heading;
	public SetPositionAction(double x, double y, double heading) {
		this.x = x;
		this.y = y;
		this.heading = heading;
	}

	@Override
	public void onStart() {
		PoseEstimator.getInstance().reset(Timer.getFPGATimestamp(),
			new Pose2d(x, y, Rotation2d.fromDegrees(heading)));
	}

	@Override
	public void onLoop() {}

	@Override
	public boolean isFinished() {
		return true;
	}

	@Override
	public void onStop() {}
}
