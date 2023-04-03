package frc.robot.actions.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.statemachine.Action;
import frc.robot.subsystems.DriveTrain;

public class AutoRobotRelEncoderAction extends Action{
    private final double m_translationXSupplier;
    private final double m_translationYSupplier;
    private final double m_rotationSupplier;
    private ChassisSpeeds speeds;
    private double desiredEncoder;

    public AutoRobotRelEncoderAction(double translationXSupplier, double translationYSupplier, double rotationSupplier) {
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;
    }

    @Override
    public void onStart() {
        DriveTrain.getInstance().setAutoState();
        DriveTrain.getInstance().setZeroDriveEncoders();
        DriveTrain.getInstance().setDesiredEncoder(desiredEncoder);

        DriveTrain.getInstance().setAutoXSupplier(m_translationXSupplier);
        DriveTrain.getInstance().setAutoYSupplier(m_translationYSupplier);
        DriveTrain.getInstance().setRobotRelBool(true);
    }

    @Override
    public void onLoop() {
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void onStop() {
    }
   
}