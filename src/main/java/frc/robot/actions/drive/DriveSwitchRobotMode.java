package frc.robot.actions.drive;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.DriveTrain;

public class DriveSwitchRobotMode extends Action {

    @Override
    public void onStart() {
        DriveTrain.getInstance().toggleRobotMode();
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
