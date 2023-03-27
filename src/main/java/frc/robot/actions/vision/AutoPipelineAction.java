package frc.robot.actions.vision;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Arm;

public class AutoPipelineAction extends Action {
	int pipeline;

    public AutoPipelineAction(int pipeline){
        this.pipeline = pipeline;
    }

	@Override
	public void onStart() {
		Arm.pipeline.setDouble(pipeline);
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
