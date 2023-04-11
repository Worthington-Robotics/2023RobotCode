package frc.robot.actions.drive;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.control.RotationalTrapController;
import frc.lib.statemachine.Action;
import frc.robot.subsystems.DriveTrain;

public class TeleGyroAction extends Action {
    public double thetaAbs;

    public TeleGyroAction(double thetaAbs) {
        this.thetaAbs = thetaAbs;
    }

    @Override
    public void onStart() {
        DriveTrain.getInstance().setGyroHeading(thetaAbs);
        // DriveTrain.getInstance().setThetaAbs(thetaAbs);
        // RotationalTrapController controller = DriveTrain.getInstance().makeNewController();
        // controller.enableToGoal(DriveTrain.getInstance().getGyroscopeRotation().getRadians(), Timer.getFPGATimestamp(), thetaAbs);
        DriveTrain.getInstance().setTeleGyroLockState();
    }

    @Override
    public void onLoop() {}

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void onStop() {
        DriveTrain.getInstance().setFieldRel();
    }
    
}