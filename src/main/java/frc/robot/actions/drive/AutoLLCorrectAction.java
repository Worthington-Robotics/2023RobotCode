package frc.robot.actions.drive;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.DriveTrain;

public class AutoLLCorrectAction extends Action {

    @Override
    public void onStart() {
        DriveTrain.getInstance().setLLCorrect();
    }

    @Override
    public void onLoop() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public void onStop() {
        DriveTrain.getInstance().setFieldRel();
    }
    
}
