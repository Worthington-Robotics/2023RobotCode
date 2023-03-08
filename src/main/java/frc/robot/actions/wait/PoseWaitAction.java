package frc.robot.actions.wait;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Manipulator;
import frc.lib.statemachine.Action;
import frc.robot.Constants;

public class PoseWaitAction extends Action{
    double currentPivotError, currentExtensionError, currentWristError;

    @Override
    public void onStart() {
        currentPivotError = Arm.getInstance().getPivotEncoderError();
        currentExtensionError = Arm.getInstance().getExtendEncoderError();
        currentWristError = Manipulator.getInstance().getWristEncoderError();
    }

    @Override
    public void onLoop() {
        currentPivotError = Arm.getInstance().getPivotEncoderError();
        currentExtensionError = Arm.getInstance().getExtendEncoderError();
        currentWristError = Manipulator.getInstance().getWristEncoderError(); 
    }

    @Override
    public boolean isFinished() {
        if(currentExtensionError < Constants.EXTENSION_ENCODER_ERROR_ACCEPTANCE
            && currentPivotError < Constants.PIVOT_ENCODER_ERROR_ACCEPTANCE
            && currentWristError < Constants.WRIST_ANGLE_ENCODER_ACCEPTANCE){
                return true;
        }
        return false;
    }

    @Override
    public void onStop() {
    }
    
}
