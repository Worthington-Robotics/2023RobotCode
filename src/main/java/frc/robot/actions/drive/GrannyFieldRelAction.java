package frc.robot.actions.drive;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain.State;

public class GrannyFieldRelAction extends Action{

    @Override
    public void onStart() {
        if(DriveTrain.getInstance().getState() == State.FieldRel){
            DriveTrain.getInstance().setGrannyFieldRel(); 
        } else {
            DriveTrain.getInstance().setFieldRel();
        }
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
