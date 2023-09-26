package frc.robot.actions.manipulator;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Manipulator;

public class ManualOutAction extends Action {

    @Override
    public void onStart() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void onLoop() {
        Manipulator.getInstance().setIntakePower(-0.6);
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public void onStop() {
        Manipulator.getInstance().setIntakePower(0);
    }
    
}
