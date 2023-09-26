package frc.robot.actions.arm;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPose;

public class FollowTrajectory extends Action {

    @Override
    public void onStart() {
        // Arm.getInstance().setFollowingTrajectory(true, false);
        Arm.getInstance().setNewTrajectoryAndFollow(ArmPose.Preset.MID);
    }

    @Override
    public void onLoop() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return true;
    }

    @Override
    public void onStop() {
        // TODO Auto-generated method stub
        
    }
    
}
