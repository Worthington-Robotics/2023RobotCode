package frc.robot.actions.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.statemachine.Action;
import frc.robot.subsystems.DriveTrain;

public class DefaultDriveCommand extends Action{
    private final DriveTrain m_drivetrainSubsystem = DriveTrain.getInstance();

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;

    public DefaultDriveCommand(DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier, DoubleSupplier rotationSupplier) {
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;
    }

    @Override
    public void onStart() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void onLoop() {
        final double x = m_translationXSupplier.getAsDouble();
        final double y = m_translationYSupplier.getAsDouble();
        final double r = m_rotationSupplier.getAsDouble();
        ChassisSpeeds speeds;
        switch (m_drivetrainSubsystem.getState()) {
            case FieldRel:
                speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    x,
                    y,
                    r,
                    m_drivetrainSubsystem.getGyroscopeRotation()
                );
                break;
            case RobotRel:
                speeds = new ChassisSpeeds(x, y, r);
                break;
            default:
                speeds = new ChassisSpeeds();
        }
        m_drivetrainSubsystem.setChassisSpeeds(speeds);
        if (x != 0.0 && y != 0.0 && r != 0.0) {
            // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        }
        
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public void onStop() {
        m_drivetrainSubsystem.setChassisSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
   
}
