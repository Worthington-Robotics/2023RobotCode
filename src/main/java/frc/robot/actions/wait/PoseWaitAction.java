package frc.robot.actions.wait;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Manipulator;
import frc.lib.statemachine.Action;
import frc.robot.Constants;

public class PoseWaitAction extends Action{

    @Override
    public void onStart() {
    }

    @Override
    public void onLoop() {
    }

    @Override
    public boolean isFinished() {
        if((Math.abs(Arm.getInstance().getLengthError()) < Constants.EXTENSION_ENCODER_ERROR_ACCEPTANCE)
            && (Math.abs(Arm.getInstance().getPivotError()) < Constants.PIVOT_ENCODER_ERROR_ACCEPTANCE)
            && (Math.abs(Manipulator.getInstance().getWristEncoderError()) < Constants.WRIST_ENCODER_ERROR)){
                return true;
        }
        return false;
    }

    @Override
    public void onStop() {
    }
    
}
