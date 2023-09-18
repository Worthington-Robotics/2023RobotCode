package frc.robot.actions.arm;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.arm.Arm;

public class FollowTrajectory extends Action {

    @Override
    public void onStart() {
        Arm.getInstance().setFollowingTrajectory(true, false);
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
