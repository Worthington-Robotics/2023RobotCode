package frc.robot.actions.vision;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.VisionLink;
import frc.robot.subsystems.VisionLink.LimelightPipeline;

public class SetPipelineAction extends Action {
	LimelightPipeline pipeline;
	public SetPipelineAction(LimelightPipeline pipeline) {
		this.pipeline = pipeline;
	}

	@Override
	public void onStart() {
		VisionLink.getInstance().setPipeline(pipeline);
	}

	@Override
	public void onLoop() {}

	@Override
	public void onStop() {}

	@Override
	public boolean isFinished() {
		return true;
	}
}