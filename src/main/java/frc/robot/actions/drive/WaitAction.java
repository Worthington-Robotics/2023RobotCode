
package frc.robot.actions.drive;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.DriveTrain;


public class WaitAction extends Action {
    
    @Override
    public void onStart() {
        DriveTrain.getInstance().setStopped();  
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
        DriveTrain.getInstance().resetEncoders();
        DriveTrain.getInstance().setStopped();
        
    }
    
}
