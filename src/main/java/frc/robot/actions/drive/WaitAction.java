
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
<<<<<<< Updated upstream:src/main/java/frc/robot/actions/drive/WaitAction.java
        return false;
=======
       return Math.abs(pitch) < 3.0;
>>>>>>> Stashed changes:src/main/java/frc/robot/actions/wait/LevelPitchWaitAction.java
    }

    @Override
    public void onStop() {
        DriveTrain.getInstance().resetEncoders();
        DriveTrain.getInstance().setStopped();
        
    }
    
}
