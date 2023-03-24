package frc.robot.actions.drive;

import frc.lib.control.ErrorChecker;
import frc.lib.statemachine.Action;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class DriveTurnAction extends Action {
    double heading;
    ErrorChecker checker = new ErrorChecker(Constants.ANGLE_ACCEPTANCE, Constants.ANGLE_PID_MINIMUM_TIME);

    public DriveTurnAction(double heading) {
        this.heading = heading;
    }

    @Override
    public void onStart() {
        DriveTrain.getInstance().setTurn(heading);
    }

    @Override
    public void onLoop() {}

    @Override
    public boolean isFinished() {
        return checker.check(DriveTrain.getInstance().getHeadingError());
    }

    @Override
    public void onStop() {
        DriveTrain.getInstance().resetEncoders();
        DriveTrain.getInstance().setOpenLoop();
    }
}
