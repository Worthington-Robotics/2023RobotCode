package frc.robot.actions.drive;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.DriveTrain;

public class GrannyFieldRelAction extends Action{

    @Override
    public void onStart() {
        DriveTrain.getInstance().setGrannyFieldRel(); 
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
        DriveTrain.getInstance().setFieldRel();
    }
    
}
