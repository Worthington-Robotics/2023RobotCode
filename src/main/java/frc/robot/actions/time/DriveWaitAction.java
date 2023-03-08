
<<<<<<<< HEAD:src/main/java/frc/robot/actions/wait/WaitAction.java
package frc.robot.actions.wait;
========
package frc.robot.actions.time;
>>>>>>>> 3f0bcf81d9c2615c6f2a78dc7ae6120af60cbf3c:src/main/java/frc/robot/actions/time/DriveWaitAction.java

import frc.lib.statemachine.Action;
import frc.robot.subsystems.DriveTrain;


public class DriveWaitAction extends Action {
    
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
