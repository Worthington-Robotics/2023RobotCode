package frc.robot.actions.vision;

import frc.lib.statemachine.Action;
import frc.robot.Constants;
import edu.wpi.first.networktables.NetworkTableInstance;

public class AutoPipelineAction extends Action {
	int pipeline;

    public AutoPipelineAction(int pipeline){
        this.pipeline = pipeline;
    }

	@Override
	public void onStart() {
        NetworkTableInstance.getDefault().getTable(Constants.LIMELIGHT_NETWORK_ID).getEntry("pipeline").setDouble(pipeline);
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
