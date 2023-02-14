package frc.robot.actions.drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.statemachine.Action;
import frc.robot.subsystems.DriveTrain;
import frc.robot.util.DebugLogger;
import frc.robot.util.DebugLogger.DebugLevel;
import frc.robot.Constants;

public class MoveForward extends Action{
    double targetDistance;
    double desiredHeading;

    public MoveForward (double targetDistance, double desiredHeading){
        this.targetDistance = targetDistance;
        this.desiredHeading = desiredHeading;
    }

    @Override
    public void onStart() {
        DriveTrain.getInstance().setDesiredHeading(desiredHeading);
        DriveTrain.getInstance().setEncoderError(targetDistance);
        DriveTrain.getInstance().setTargetDistance(targetDistance);
        DriveTrain.getInstance().setMoveForward();
    }

    @Override
    public void onLoop() {}

    @Override
    public boolean isFinished() {
        double error = Math.abs(DriveTrain.getInstance().getEncoderError());
        return (error < Constants.DRIVE_FORWARD_ACCEPTED_ERROR);
    }

    @Override
    public void onStop() {
        DriveTrain.getInstance().setStopped();
    }
}
