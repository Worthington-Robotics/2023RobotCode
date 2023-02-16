package frc.robot.actions.drive;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.statemachine.Action;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class DriveTurnAction extends Action {
    double heading;
    double startTime = Timer.getFPGATimestamp();

    public DriveTurnAction(double heading) {
        this.heading = heading;
    }

    @Override
    public void onStart() {
        DriveTrain.getInstance().setTurning(heading);
    }

    @Override
    public void onLoop() {}

    @Override
    public boolean isFinished() {
        return Math.abs(DriveTrain.getInstance().getHeadingError()) < Constants.ANGLE_ACCEPTANCE
            && Timer.getFPGATimestamp() - startTime > Constants.ANGLE_PID_MINIMUM_TIME;
    }

    @Override
    public void onStop() {
        DriveTrain.getInstance().resetEncoders();
        DriveTrain.getInstance().setStopped();
    }
}
