package frc.robot.actions.arm;

import frc.lib.physics.DriveCharacterization.AccelerationDataPoint;
import frc.lib.statemachine.Action;
import frc.robot.subsystems.Arm;

public class PinToggleAction extends Action{

    @Override
    public void onStart() {
        Arm.getInstance().setPin();
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
        Arm.getInstance().clearPin();
    }
    
}
