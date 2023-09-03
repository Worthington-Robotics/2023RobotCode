package frc.robot.actions.arm;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Manipulator;
import frc.robot.subsystems.arm.Arm.ArmPose;

public class ArmZeroPose extends Action {
    ArmPose pose = ArmPose.ZERO;

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