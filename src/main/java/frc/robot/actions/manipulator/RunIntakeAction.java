package frc.robot.actions.manipulator;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Arm;


public class RunIntakeAction extends Action {
    // The speed to run the intake at
    double power;
    boolean cancel;
    
    public RunIntakeAction(double power) {
        this.power = power;
        cancel = true;
    }

    public RunIntakeAction(double power, boolean cancel) {
        this.power = power;
        this.cancel = cancel;
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
        Manipulator.getInstance().setIntakePower(0);
    }

    @Override
    public boolean isFinished() {
        return (cancel && (power > 0) && Manipulator.getInstance().isObject());
    }
}