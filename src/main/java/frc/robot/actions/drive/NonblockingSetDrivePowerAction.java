package frc.robot.actions.drive;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.DriveTrain;

public class NonblockingSetDrivePowerAction extends Action {
    double motorPower;
    double desiredHeading;

    public NonblockingSetDrivePowerAction (double desiredheading, double motorPower){
        this.motorPower = motorPower;
        this.desiredHeading = desiredheading;
    }

    @Override
    public void onStart() {
        DriveTrain.getInstance().setDesiredHeading(desiredHeading);
        DriveTrain.getInstance().setMotorPower(motorPower);
    }

    @Override
    public void onLoop() {}

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void onStop() {
    }
}