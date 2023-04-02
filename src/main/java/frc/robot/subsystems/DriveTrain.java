package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.loops.ILooper;
import frc.lib.loops.Loop;
import frc.lib.util.HIDHelper;
import frc.robot.Constants;
import frc.robot.actions.drive.DefaultDriveCommand;

public class DriveTrain extends Subsystem {
    private static DriveTrain instance = new DriveTrain();
    public static DriveTrain getInstance() { return instance; }
    private DriveTrainIO periodic = new DriveTrainIO();

    private final PigeonIMU m_pigeon = new PigeonIMU(0);
    public SwerveDriveOdometry odometry;

    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
		// Front left
		new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
		// Front right
		new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
		// Back left
		new Translation2d(-Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
		// Back right
		new Translation2d(-Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0)
	);

    private final SwerveModule m_frontLeftModule;
    private final SwerveModule m_frontRightModule;
    private final SwerveModule m_backLeftModule;
    private final SwerveModule m_backRightModule;

    public static final double MAX_VOLTAGE = 9.5;

    public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380 / 60.0 *
    SdsModuleConfigurations.MK4_L3.getDriveReduction() *
    SdsModuleConfigurations.MK4_L3.getWheelDiameter() * Math.PI;

    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
    Math.hypot(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0);

    public enum State {
        FieldRel,
        RobotRel
    }

    public class DriveTrainIO {
        public State state = State.RobotRel;
        public SwerveModuleState[] states = {new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()};
        public double XboxLeftY;
        public double XboxLeftX;
        public double XboxRightX;
        public ChassisSpeeds speeds;
    }

    private DriveTrain() {
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        
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
        setZeroDriveEncoders();
        setGyroZero();

        odometry = new SwerveDriveOdometry(m_kinematics, new Rotation2d(), new SwerveModulePosition[] {
            new SwerveModulePosition(0.0, Rotation2d.fromDegrees(m_frontLeftModule.getSteerAngle())),
            new SwerveModulePosition(0.0, Rotation2d.fromDegrees(m_frontRightModule.getSteerAngle())),
            new SwerveModulePosition(0.0, Rotation2d.fromDegrees(m_backLeftModule.getSteerAngle())),
            new SwerveModulePosition(0.0, Rotation2d.fromDegrees(m_backRightModule.getSteerAngle())),
        });
    }

    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                periodic.speeds = new ChassisSpeeds(0.0, 0.0, 0.0);
                periodic.states = m_kinematics.toSwerveModuleStates(periodic.speeds);
            }
            @Override
            public void onLoop(double timestamp) {
                periodic.states = m_kinematics.toSwerveModuleStates(periodic.speeds);
		        SwerveDriveKinematics.desaturateWheelSpeeds(periodic.states, MAX_VELOCITY_METERS_PER_SECOND);

                odometry.update(getGyroscopeRotation(), new SwerveModulePosition[] {
                    new SwerveModulePosition(m_frontLeftModule.getDriveEncoder() /  Constants.DRIVE_ENCODER_TO_METERS, Rotation2d.fromRadians(m_frontLeftModule.getSteerAngle())),
                    new SwerveModulePosition(m_frontRightModule.getDriveEncoder() / Constants.DRIVE_ENCODER_TO_METERS, Rotation2d.fromRadians(m_frontLeftModule.getSteerAngle())),
                    new SwerveModulePosition(m_backLeftModule.getDriveEncoder() /   Constants.DRIVE_ENCODER_TO_METERS, Rotation2d.fromRadians(m_frontLeftModule.getSteerAngle())),
                    new SwerveModulePosition(m_backRightModule.getDriveEncoder() /  Constants.DRIVE_ENCODER_TO_METERS, Rotation2d.fromRadians(m_frontLeftModule.getSteerAngle())),
                });

                final double x = periodic.XboxLeftX;
                final double y = periodic.XboxLeftY;
                final double r = periodic.XboxRightX;
                ChassisSpeeds speeds;
                switch (periodic.state) {
                    case FieldRel:
                    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    x,
                    y,
                    r,
                    getGyroscopeRotation()
                );
                break;
            case RobotRel:
                speeds = new ChassisSpeeds((x * 4.0), (y * 4.0), (r * 360.0));
                break;
            default:
                speeds = new ChassisSpeeds();
        }
        setChassisSpeeds(speeds);
        if (x != 0.0 && y != 0.0 && r != 0.0) {
            // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        }

            }
            @Override
            public void onStop(double timestamp) {
                setChassisSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0));
            }
        });
    }

    public void setGyroZero() {
		m_pigeon.setFusedHeading(0.0);
	}

	public Rotation2d getGyroscopeRotation() {
		return Rotation2d.fromDegrees(m_pigeon.getFusedHeading());
	}

    public State getState() {
        return periodic.state;
    }

	public void setZeroDriveEncoders() {
		m_frontLeftModule.resetDriveEncoder();
		m_frontRightModule.resetDriveEncoder();
		m_backLeftModule.resetDriveEncoder();
		m_backRightModule.resetDriveEncoder();
	}

	public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
		periodic.speeds = chassisSpeeds;
	}
    

    public void readPeriodicInputs() {
        double LeftX = -Constants.XBOX.getLeftX();
        double LeftY = Constants.XBOX.getLeftY();
        double RightX = Constants.XBOX.getRightX();

        if (Math.abs(LeftX) < Constants.XBOX_DEADZONE) {
            periodic.XboxLeftX = 0.0;
        } else {
            periodic.XboxLeftX = LeftX;
        }

        if (Math.abs(LeftY) < Constants.XBOX_DEADZONE) {
            periodic.XboxLeftY = 0.0;
        } else {
            periodic.XboxLeftY = LeftY;
        }

        if (Math.abs(RightX) < Constants.XBOX_DEADZONE) {
            periodic.XboxRightX = 0.0;
        } else {
            periodic.XboxRightX = RightX;
        }


    }

    public void writePeriodicOutputs() {
        m_frontLeftModule.set(periodic.states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, periodic.states[0].angle.getRadians());
		m_frontRightModule.set(periodic.states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, periodic.states[1].angle.getRadians());
		m_backLeftModule.set(periodic.states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, periodic.states[2].angle.getRadians());
		m_backRightModule.set(periodic.states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, periodic.states[3].angle.getRadians());
    }

    public void outputTelemetry() {
        final Pose2d odomPose = odometry.getPoseMeters();
		final double heading = m_pigeon.getFusedHeading();
		SmartDashboard.putNumberArray("Drive/Odometry",
			new double[] {
				odomPose.getX(),
				odomPose.getY(),
				Math.toRadians(heading)
			}
		);
		SmartDashboard.putNumberArray("Drive/Swerve", new double[] {
			m_frontLeftModule.getSteerAngle(), m_frontLeftModule.getDriveVelocity(),
			m_frontRightModule.getSteerAngle(), m_frontRightModule.getDriveVelocity(),
			m_backLeftModule.getSteerAngle(), m_backLeftModule.getDriveVelocity(),
			m_backRightModule.getSteerAngle(), m_backRightModule.getDriveVelocity(),
		});
		SmartDashboard.putNumberArray("Drive/Swerve Setpoint", new double[] {
			m_frontLeftModule.getDesiredSteerAngle(), periodic.states[0].speedMetersPerSecond,
			m_frontRightModule.getDesiredSteerAngle(), periodic.states[1].speedMetersPerSecond,
			m_backLeftModule.getDesiredSteerAngle(), periodic.states[2].speedMetersPerSecond,
			m_backRightModule.getDesiredSteerAngle(), periodic.states[3].speedMetersPerSecond,
		});
		SmartDashboard.putNumberArray("Drive/Module Encoders", new double[] {
			m_frontLeftModule.getDriveEncoder(),
			m_frontRightModule.getDriveEncoder(),
			m_backLeftModule.getDriveEncoder(),
			m_backRightModule.getDriveEncoder(),
		});
		SmartDashboard.putString("Drive/Mode", periodic.state.toString());
        SmartDashboard.putNumberArray("Drive/Swerve Setpoint Joy", new double[] {
            periodic.XboxLeftX, periodic.XboxLeftY, periodic.XboxRightX
        });
    }

    public void reset() {
        setChassisSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0));
        setGyroZero();
    }
}