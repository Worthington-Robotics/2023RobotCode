package frc.robot.actions.drive;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.DriveTrain;

public class DriveZeroGyro extends Action {

    @Override
    public void onStart() {
        DriveTrain.getInstance().setGyroZero();
        
    }

    @Override
    public void onLoop() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void onStop() {
        // TODO Auto-generated method stub
        
    }
    
}
