package frc.robot.actions.vision;

import frc.lib.statemachine.Action;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmPose;
import edu.wpi.first.networktables.NetworkTableInstance;

public class SetPipelineAction extends Action {

	@Override
	public void onStart() {
		if(Arm.pipeline.getDouble(-1) != 0 && Arm.getInstance().getPose() == ArmPose.MID){
			Arm.pipeline.setDouble(0);
		} else if(Arm.pipeline.getDouble(-1) != 1 && Arm.getInstance().getPose() == ArmPose.HIGH){
			Arm.pipeline.setDouble(1);
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