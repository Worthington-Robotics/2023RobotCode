package frc.robot.actions.arm;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Arm.ArmPose;

public class TwoPoseAction extends Action {

    ArmPose beginPose, endPose;

    public TwoPoseAction(ArmPose beginPose, ArmPose endPose) {
        this.beginPose = beginPose;
        this.endPose = endPose;
    }

    @Override
    public void onStart() {
        Arm.getInstance().setPose(beginPose);
        Manipulator.getInstance().resWrist();
        if(beginPose == ArmPose.HIGH) {
			Arm.pipeline.setDouble(1);
        } else {
			Arm.pipeline.setDouble(0);
        }
    }

    @Override
    public void onLoop() {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void onStop() {
        Arm.getInstance().setPose(endPose);
        if(endPose == ArmPose.HIGH) {
			Arm.pipeline.setDouble(1);
        } else {
			Arm.pipeline.setDouble(0);
        }
    }
    
}
