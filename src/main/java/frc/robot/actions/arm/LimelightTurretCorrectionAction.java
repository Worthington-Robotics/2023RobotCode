package frc.robot.actions.arm;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.lib.control.ErrorChecker;
import frc.lib.statemachine.Action;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.VisionLink;

public class LimelightTurretCorrectionAction extends Action {
    ErrorChecker checker = new ErrorChecker(Constants.LIMELIGHT_ANGLE_ACCEPTANCE,
        Constants.LIMELIGHT_ANGLE_PID_MINIMUM_TIME);

    @Override
    public void onStart() {
        double heading = Arm.getInstance().getTurretAngle();
        Arm.getInstance().setLimelightCorrection();
        Arm.getInstance().setDesiredTurret(heading - VisionLink.getInstance().getTurretOffset());
        Lights.getInstance().setLimelightTargetColor();
    }

    @Override
    public void onLoop() {}

    @Override
    public boolean isFinished() {
        return checker.check(Arm.getInstance().getTurretError() / Constants.TURRET_ENCODER_PER_DEGREE);
    }

    @Override
    public void onStop() {
        Arm.getInstance().setLimelightCorrectionOff();
        Lights.getInstance().setRainbowLights();
    }
}
