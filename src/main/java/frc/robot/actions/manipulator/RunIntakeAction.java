package frc.robot.actions.manipulator;

import frc.lib.statemachine.Action;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Manipulator;


public class RunIntakeAction extends Action {
    // The speed to run the intake at
    double power = Constants.Arm.ANYTHING_OUT_POWER;
    boolean isRelease = false;

    @Override
    public void onStart() {
        Manipulator.getInstance().setIntakePower(power);
    }

    @Override
    public void onLoop() {}

    @Override
    public void onStop() {
        if(isRelease){ //releasing
            Manipulator.getInstance().setIntakePower(0);
            Manipulator.getInstance().setGamePiece(false);
        } else { //intaking
            Manipulator.getInstance().setIntakePower(0.1);
            Manipulator.getInstance().setGamePiece(true);
        }
    }

    @Override
    public boolean isFinished() {
        if(Manipulator.getInstance().getAutoBool() && !isRelease){
            return Manipulator.getInstance().isObject();
        }
        return false;
    }
}