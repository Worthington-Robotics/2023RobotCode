package frc.robot.actions.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.statemachine.Action;
import frc.robot.subsystems.DriveTrain;

public class NonblockingDrivePowerAction extends Action{

    private final double m_translationXSupplier;
    private final double m_translationYSupplier;
    private final double m_rotationSupplier;
    private ChassisSpeeds speeds;

    public NonblockingDrivePowerAction(double translationXSupplier, double translationYSupplier, double rotationSupplier) {
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;
    }

    @Override
    public void onStart() {
        DriveTrain.getInstance().setTimeAutoState();
        
    }

    @Override
    public void onLoop() {
        final double x = m_translationXSupplier;
        final double y = m_translationYSupplier;
        final double r = m_rotationSupplier;

        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            x,
            y,
            r,
            DriveTrain.getInstance().getGyroscopeRotation()
        );


        DriveTrain.getInstance().setChassisSpeeds(speeds);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void onStop() {
    }
    
}
