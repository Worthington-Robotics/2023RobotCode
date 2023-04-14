package frc.robot.actions.manipulator;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Lights.State;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Lights;


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
        Arm.snapshots.setNumber(1);
    }

    @Override
    public void onLoop() {}

    @Override
    public void onStop() {
        Arm.snapshots.setNumber(0);
        if(isRelease){ //releasing
            Manipulator.getInstance().setIntakePower(0);
            Manipulator.getInstance().setGamePiece(false);
            Lights.getInstance().setLightState(State.LIGHTS_RAINBOW);
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