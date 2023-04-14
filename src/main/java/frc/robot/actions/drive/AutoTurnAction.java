package frc.robot.actions.drive;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.control.RotationalTrapController;
import frc.lib.statemachine.Action;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class AutoTurnAction extends Action {
    public double thetaAbs;

    public AutoTurnAction(double thetaAbs) {
        this.thetaAbs = thetaAbs;
    }

    @Override
    public void onStart() {
        DriveTrain.getInstance().setThetaAbs(thetaAbs);
        RotationalTrapController controller = DriveTrain.getInstance().makeNewController();
        controller.enableToGoal(DriveTrain.getInstance().getGyroscopeRotation().getRadians(), Timer.getFPGATimestamp(), thetaAbs);
        DriveTrain.getInstance().setAutoTurnState();
    }

    @Override
    public void onLoop() {}

    @Override
    public boolean isFinished() {
        double goalHeading = thetaAbs;
        double currentHeading = DriveTrain.getInstance().getGyroscopeRotation().getRadians();
        return Math.abs(Math.abs(goalHeading) - Math.abs(currentHeading)) < (Constants.DRIVE_TURN_ERROR);
    }

    @Override
    public void onStop() {}
    
}