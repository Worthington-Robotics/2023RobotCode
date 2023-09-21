package frc.robot.actions.drive;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.SwerveDrive;

public class ZeroGyro extends Action {

    @Override
    public void onStart() {
        SwerveDrive.getInstance().zeroGyroHeading();
        
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
