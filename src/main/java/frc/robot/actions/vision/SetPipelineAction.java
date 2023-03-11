package frc.robot.actions.vision;

import frc.lib.statemachine.Action;
<<<<<<< HEAD
import frc.robot.Constants;
import frc.robot.subsystems.VisionLink;
import frc.robot.subsystems.VisionLink.LimelightPipeline;
import edu.wpi.first.networktables.NetworkTableInstance;
=======
import frc.robot.subsystems.VisionLink.LimelightPipeline;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
>>>>>>> c2aec83e56e319ffe7203bd744b1e0a85f51523f

public class SetPipelineAction extends Action {
	LimelightPipeline pipeline;
	int pipelineVal;
<<<<<<< HEAD
=======
	public SetPipelineAction(LimelightPipeline pipeline) {
		this.pipeline = pipeline;
	}
>>>>>>> c2aec83e56e319ffe7203bd744b1e0a85f51523f

	@Override
	public void onStart() {
		if(pipeline == LimelightPipeline.High){
<<<<<<< HEAD
			pipelineVal = Constants.LIMELIGHT_LOW_PIPELINE;
			NetworkTableInstance.getDefault().getTable(Constants.LIMELIGHT_NETWORK_ID).getEntry("pipeline").setDouble(pipelineVal);
			pipeline = LimelightPipeline.Low;
		} else {
			pipelineVal = Constants.LIMELIGHT_HIGH_PIPELINE;
			NetworkTableInstance.getDefault().getTable(Constants.LIMELIGHT_NETWORK_ID).getEntry("pipeline").setDouble(pipelineVal);
			pipeline = LimelightPipeline.High;
		}
=======
			pipelineVal = Constants.LIMELIGHT_HIGH_PIPELINE;
		} else {
			pipelineVal = Constants.LIMELIGHT_LOW_PIPELINE;
		}
		NetworkTableInstance.getDefault().getTable(Constants.LIMELIGHT_NETWORK_ID).getEntry("pipeline").setNumber(pipelineVal);
>>>>>>> c2aec83e56e319ffe7203bd744b1e0a85f51523f
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