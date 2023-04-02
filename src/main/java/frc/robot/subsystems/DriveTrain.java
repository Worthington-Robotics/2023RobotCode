package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.lib.loops.ILooper;
import frc.lib.loops.Loop;
import frc.lib.util.HIDHelper;
import frc.robot.Constants;

public class DriveTrain extends Subsystem {
    private static DriveTrain instance = new DriveTrain();
    public static DriveTrain getInstance() { return instance; }
    private DriveTrainIO periodic = new DriveTrainIO();

    private final PigeonIMU m_pigeon = new PigeonIMU(0);
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
    public SwerveDriveOdometry odometry;
    private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    private final SwerveModule m_frontLeftModule;
    private final SwerveModule m_frontRightModule;
    private final SwerveModule m_backLeftModule;
    private final SwerveModule m_backRightModule;

    public enum State {
    }

    public class DriveTrainIO {
        public State state;
    }

    private DriveTrain() {
        
    m_frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
        // This parameter is optional, but will allow you to see the current state of the module on the dashboard.
        tab.getLayout("Front Left Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(0, 0),
        // This can either be STANDARD or FAST depending on your gear configuration
        Mk4SwerveModuleHelper.GearRatio.L3,
        // This is the ID of the drive motor
        Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
        // This is the ID of the steer motor
        Constants.FRONT_LEFT_MODULE_STEER_MOTOR,
        // This is the ID of the steer encoder
        Constants.FRONT_LEFT_MODULE_STEER_ENCODER,
        // This is how much the steer encoder is offset from true zero (In our case, zero is facing straight forward)
        Constants.FRONT_LEFT_MODULE_STEER_OFFSET
    );

    m_frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
        tab.getLayout("Front Right Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(2, 0),
        Mk4SwerveModuleHelper.GearRatio.L3,
        Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
        Constants.FRONT_RIGHT_MODULE_STEER_MOTOR,
        Constants.FRONT_RIGHT_MODULE_STEER_ENCODER,
        Constants.FRONT_RIGHT_MODULE_STEER_OFFSET
    );

    m_backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
        tab.getLayout("Back Left Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(4, 0),
        Mk4SwerveModuleHelper.GearRatio.L3,
       Constants.BACK_LEFT_MODULE_DRIVE_MOTOR,
       Constants.BACK_LEFT_MODULE_STEER_MOTOR,
       Constants.BACK_LEFT_MODULE_STEER_ENCODER,
       Constants.BACK_LEFT_MODULE_STEER_OFFSET
    );

    m_backRightModule = Mk4SwerveModuleHelper.createFalcon500(
        tab.getLayout("Back Right Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(6, 0),
        Mk4SwerveModuleHelper.GearRatio.L3,
        Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
        Constants.BACK_RIGHT_MODULE_STEER_MOTOR,
        Constants.BACK_RIGHT_MODULE_STEER_ENCODER,
        Constants.BACK_RIGHT_MODULE_STEER_OFFSET
    );

    }

    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
            }
            @Override
            public void onLoop(double timestamp) {
            }
            @Override
            public void onStop(double timestamp) {

            }
        });
    }
    

    public void readPeriodicInputs() {
    }

    public void writePeriodicOutputs() {
    }

    public void outputTelemetry() {
    }

    public void reset() {
    }
}