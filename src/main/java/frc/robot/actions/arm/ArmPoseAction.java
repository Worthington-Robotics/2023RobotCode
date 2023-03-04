package frc.robot.actions.arm;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmPose;

public class ArmPoseAction extends Action {

    ArmPose pose;

    public ArmPoseAction(ArmPose pose) {
        this.pose = pose;
    }

    @Override
    public void onStart() {
        Arm.getInstance().setPose(pose);
    }

    @Override
    public void onLoop() {
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void onStop() {
    }
    
}
