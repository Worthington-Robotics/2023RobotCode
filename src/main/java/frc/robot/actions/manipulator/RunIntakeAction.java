package frc.robot.actions.manipulator;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Manipulator;


public class RunIntakeAction extends Action {
    // The speed to run the intake at
    double power;
    boolean isRelease;
    
    public RunIntakeAction(double power) { //releasing
        this.power = power;
        isRelease = true;
    }

    public RunIntakeAction(double power, boolean isRelease) { //intaking
        this.power = power;
        this.isRelease = isRelease;
    }

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