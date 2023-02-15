package frc.robot.actions.drive;

import frc.lib.statemachine.Action;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class DriveTurn extends Action {
    double heading;
    public DriveTurn(double heading) {
        this.heading = heading;
    }

    @Override
    public void onStart() {
        DriveTrain.getInstance().setDesiredHeading(heading);
        DriveTrain.getInstance().setHeadingError(heading);
        DriveTrain.getInstance().setTurning();
    }

    @Override
    public void onLoop() {}

    @Override
    public boolean isFinished() {
        return Math.abs(DriveTrain.getInstance().getHeadingError()) < Constants.ANGLE_ACCEPTANCE;
    }

    @Override
    public void onStop() {
        DriveTrain.getInstance().resetEncoders();
        DriveTrain.getInstance().setStopped();
    }
}
