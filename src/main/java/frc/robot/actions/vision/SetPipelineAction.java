package frc.robot.actions.vision;

import frc.lib.statemachine.Action;
import frc.robot.Constants;
import frc.robot.subsystems.VisionLink;
import frc.robot.subsystems.VisionLink.LimelightPipeline;
import edu.wpi.first.networktables.NetworkTableInstance;

public class SetPipelineAction extends Action {
	LimelightPipeline pipeline;
	int pipelineVal;

	@Override
	public void onStart() {
		if(pipeline == LimelightPipeline.High){
			pipelineVal = Constants.LIMELIGHT_LOW_PIPELINE;
			NetworkTableInstance.getDefault().getTable(Constants.LIMELIGHT_NETWORK_ID).getEntry("pipeline").setDouble(pipelineVal);
			pipeline = LimelightPipeline.Low;
		} else {
			pipelineVal = Constants.LIMELIGHT_HIGH_PIPELINE;
			NetworkTableInstance.getDefault().getTable(Constants.LIMELIGHT_NETWORK_ID).getEntry("pipeline").setDouble(pipelineVal);
			pipeline = LimelightPipeline.High;
		}
	}

	@Override
	public void onLoop() {}

	@Override
	public void onStop() {
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}