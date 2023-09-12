package frc.robot.actions.arm;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmPose;

public class ArmSlidePose extends Action {
    ArmPose pose = ArmPose.SLIDE;

    @Override
    public void onStart() {
        Arm.getInstance().setPose(pose);
        Manipulator.getInstance().resWrist();
        Arm.getInstance().resPivot();
    }

    @Override
    public void onLoop() {}

    @Override
    public boolean isFinished() {
        return Arm.getInstance().getAccepted();
    }

    @Override
    public void onStop() {}
    
}