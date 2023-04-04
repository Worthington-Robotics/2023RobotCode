package frc.robot.actions.drive;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.DriveTrain;

public class ToggleChargeStationLockAction extends Action {
    DriveTrain.State state;
    @Override
    public void onStart() {
        state = DriveTrain.getInstance().getState();
        if (state == DriveTrain.State.ChargeStationLock) {
            DriveTrain.State previousState = DriveTrain.getInstance().getPreviousState();
            if(previousState == DriveTrain.State.FieldRel) {
                DriveTrain.getInstance().setFieldRel();
            } else if (previousState == DriveTrain.State.RobotRel) {
                DriveTrain.getInstance().setFieldRel();
            } else if (previousState == DriveTrain.State.AutoControlled) {
                DriveTrain.getInstance().setAutoState();
            }
        } else {
            DriveTrain.getInstance().setPreviousState(state);
            DriveTrain.getInstance().setChargeStationLock();
        }
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
