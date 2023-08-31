package frc.robot.subsystems;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.loops.ILooper;
import frc.lib.loops.Loop;
import frc.robot.Constants;

public class SwerveDrive extends Subsystem {
    private static SwerveDrive instance = new SwerveDrive();
    private SwerveDriveIO periodic = new SwerveDriveIO();
    private final PigeonIMU pigeon = new PigeonIMU(0);

    public static SwerveDrive getInstance() {
        return instance;
    }

    public enum State {
        FieldRel,
        RobotRel,
        SlowFieldRel,
        PathPlanner,
        ChargeStationLock
    }
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("SwerveDrive");
    DoubleArrayPublisher poseEstimatorPub = table.getDoubleArrayTopic("Pose Estimator").publish();
    DoubleArrayPublisher swervePub = table.getDoubleArrayTopic("Swerve").publish();
    DoubleArrayPublisher setpointPub = table.getDoubleArrayTopic("Setpoint").publish();
    DoubleArrayPublisher encoderPub = table.getDoubleArrayTopic("Encoders").publish();
    StringPublisher statePub = table.getStringTopic("State").publish();
            
    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    public class SwerveDriveIO {
        public State state = State.FieldRel;
        public SwerveDrivePoseEstimator poseEstimator;
        public Pose2d simPoseEstimator = new Pose2d();
        public SwerveModuleState[] states = {new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()};
        public ChassisSpeeds speeds = new ChassisSpeeds();
        public ChassisSpeeds PPspeeds = new ChassisSpeeds();
        public double[] lastSwervePositionsRad = {0.0, 0.0, 0.0, 0.0};
        public double XboxLeftX;
        public double XboxLeftY;
        public double XboxRightX;
        public SwerveModulePosition[] simulatedModulePositions = {
            new SwerveModulePosition(0, new Rotation2d()),
            new SwerveModulePosition(0, new Rotation2d()),
            new SwerveModulePosition(0, new Rotation2d()),
            new SwerveModulePosition(0, new Rotation2d()),
        };
    }

    public SwerveDrive() {
        //Create all of our swerve modules, with their ids and gear ratio constants
        frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
            // This can either be STANDARD or FAST depending on your gear configuration
            Mk4SwerveModuleHelper.GearRatio.L3,
            // This is the ID of the drive motor
            Constants.DriveTrain.FRONT_LEFT_MODULE_DRIVE_MOTOR,
            // This is the ID of the steer motor
            Constants.DriveTrain.FRONT_LEFT_MODULE_STEER_MOTOR,
            // This is the ID of the steer encoder
            Constants.DriveTrain.FRONT_LEFT_MODULE_STEER_ENCODER,
            // This is how much the steer encoder is offset from true zero (In our case,
            // zero is facing straight forward)
            Constants.DriveTrain.FRONT_LEFT_MODULE_STEER_OFFSET);

        frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
                Mk4SwerveModuleHelper.GearRatio.L3,
                Constants.DriveTrain.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                Constants.DriveTrain.FRONT_RIGHT_MODULE_STEER_MOTOR,
                Constants.DriveTrain.FRONT_RIGHT_MODULE_STEER_ENCODER,
                Constants.DriveTrain.FRONT_RIGHT_MODULE_STEER_OFFSET);
        backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                Mk4SwerveModuleHelper.GearRatio.L3,
                Constants.DriveTrain.BACK_LEFT_MODULE_DRIVE_MOTOR,
                Constants.DriveTrain.BACK_LEFT_MODULE_STEER_MOTOR,
                Constants.DriveTrain.BACK_LEFT_MODULE_STEER_ENCODER,
                Constants.DriveTrain.BACK_LEFT_MODULE_STEER_OFFSET);
        backRightModule = Mk4SwerveModuleHelper.createFalcon500(
                Mk4SwerveModuleHelper.GearRatio.L3,
                Constants.DriveTrain.BACK_RIGHT_MODULE_DRIVE_MOTOR,
                Constants.DriveTrain.BACK_RIGHT_MODULE_STEER_MOTOR,
                Constants.DriveTrain.BACK_RIGHT_MODULE_STEER_ENCODER,
                Constants.DriveTrain.BACK_RIGHT_MODULE_STEER_OFFSET);

        zeroDriveEncoders();
        zeroGyroHeading();

        periodic.poseEstimator = new SwerveDrivePoseEstimator(Constants.DriveTrain.SWERVE_KINEMATICS, getGyroRotation(), new SwerveModulePosition[] {
            new SwerveModulePosition(0.0, Rotation2d.fromDegrees(frontLeftModule.getSteerAngle())),
            new SwerveModulePosition(0.0, Rotation2d.fromDegrees(frontRightModule.getSteerAngle())),
            new SwerveModulePosition(0.0, Rotation2d.fromDegrees(backLeftModule.getSteerAngle())),
            new SwerveModulePosition(0.0, Rotation2d.fromDegrees(backRightModule.getSteerAngle()))
        }, new Pose2d());
    }

    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                periodic.speeds = new ChassisSpeeds(0.0, 0.0, 0.0);
            }

            @Override
            public void onLoop(double timestamp) {
                periodic.states = Constants.DriveTrain.SWERVE_KINEMATICS.toSwerveModuleStates(periodic.speeds);
                SwerveDriveKinematics.desaturateWheelSpeeds(periodic.states, Constants.DriveTrain.SWERVE_MAX_VELOCITY_METERS_PER_SECOND);

                
                if(RobotBase.isSimulation()) { //Essentially simulates our swerve drive. Not physically accurate. Only use to get a general sense of what may happen.
                    SwerveModuleState[] simStates = Constants.DriveTrain.SWERVE_KINEMATICS.toSwerveModuleStates(periodic.speeds);
                    periodic.simulatedModulePositions[0] = new SwerveModulePosition(periodic.simulatedModulePositions[0].distanceMeters + (simStates[0].speedMetersPerSecond/50.0), simStates[0].angle);
                    periodic.simulatedModulePositions[1] = new SwerveModulePosition(periodic.simulatedModulePositions[1].distanceMeters + (simStates[1].speedMetersPerSecond/50.0), simStates[1].angle);
                    periodic.simulatedModulePositions[2] = new SwerveModulePosition(periodic.simulatedModulePositions[2].distanceMeters + (simStates[2].speedMetersPerSecond/50.0), simStates[2].angle);
                    periodic.simulatedModulePositions[3] = new SwerveModulePosition(periodic.simulatedModulePositions[3].distanceMeters + (simStates[3].speedMetersPerSecond/50.0), simStates[3].angle);
                    periodic.poseEstimator.update(new Rotation2d(), periodic.simulatedModulePositions);
                    //periodic.poseEstimator.getEstimatedPosition().exp(new Twist2d(periodic.speeds.vxMetersPerSecond/50.0, periodic.speeds.vyMetersPerSecond/50.0, periodic.speeds.omegaRadiansPerSecond/50.0)).getRotation()
                } else {
                    periodic.poseEstimator.update(getGyroRotation(), getSwerveModulePositions());
                }

                // SwerveModuleState[] measuredDiff = new SwerveModuleState[4];
                // measuredDiff[0] = new SwerveModuleState((frontLeftModule.getDesiredSteerAngle() - periodic.lastSwervePositionsRad[0]) * SdsModuleConfigurations.MK4_L3.getWheelDiameter(), new Rotation2d(frontLeftModule.getDesiredSteerAngle()));
                // measuredDiff[1] = new SwerveModuleState((frontRightModule.getDesiredSteerAngle() - periodic.lastSwervePositionsRad[0]) * SdsModuleConfigurations.MK4_L3.getWheelDiameter(), new Rotation2d(frontRightModule.getDesiredSteerAngle()));
                // measuredDiff[2] = new SwerveModuleState((backLeftModule.getDesiredSteerAngle() - periodic.lastSwervePositionsRad[0]) * SdsModuleConfigurations.MK4_L3.getWheelDiameter(), new Rotation2d(backLeftModule.getDesiredSteerAngle()));
                // measuredDiff[3] = new SwerveModuleState((backRightModule.getDesiredSteerAngle() - periodic.lastSwervePositionsRad[0]) * SdsModuleConfigurations.MK4_L3.getWheelDiameter(), new Rotation2d(backRightModule.getDesiredSteerAngle()));
                // periodic.lastSwervePositionsRad[0] = frontLeftModule.getDesiredSteerAngle();
                // periodic.lastSwervePositionsRad[1] = frontRightModule.getDesiredSteerAngle();
                // periodic.lastSwervePositionsRad[2] = backLeftModule.getDesiredSteerAngle();
                // periodic.lastSwervePositionsRad[3] = backRightModule.getDesiredSteerAngle();
                // ChassisSpeeds chassisStateDiff = Constants.DriveTrain.SWERVE_KINEMATICS.toChassisSpeeds(measuredDiff);
                // periodic.simPoseEstimator = periodic.simPoseEstimator.exp(new Twist2d(chassisStateDiff.vxMetersPerSecond, chassisStateDiff.vyMetersPerSecond, chassisStateDiff.omegaRadiansPerSecond));

                ChassisSpeeds speedsToApply = new ChassisSpeeds();
                switch (periodic.state) {
                    case FieldRel:
                        speedsToApply = ChassisSpeeds.fromFieldRelativeSpeeds((periodic.XboxLeftX * Constants.DriveTrain.DRIVE_XY_MULTIPLIER),(periodic.XboxLeftY * Constants.DriveTrain.DRIVE_XY_MULTIPLIER),(periodic.XboxRightX * Constants.DriveTrain.DRIVE_ROTATION_MULTIPLIER), getPoseEstimatorRotation()); //TODO: Expiriment with pose estimator position
                    break;
                    case PathPlanner:
                        speedsToApply = periodic.PPspeeds;
                    break;
                    default:
                        speedsToApply = new ChassisSpeeds();
                }
                setChassisSpeeds(speedsToApply);
            }

            @Override
            public void onStop(double timestamp) {}
            
        });
    }

    @Override
    public void readPeriodicInputs() {
        double LeftX = -Constants.Joysticks.XBOX.getLeftY();
        double LeftY = -Constants.Joysticks.XBOX.getLeftX();
        double RightX = -Constants.Joysticks.XBOX.getRightX();
        if (Math.abs(LeftX) < Constants.Joysticks.XBOX_DEADZONE) {
            periodic.XboxLeftX = 0.0;
        } else {
            periodic.XboxLeftX = LeftX;
        }
        if(Math.abs(LeftY) < Constants.Joysticks.XBOX_DEADZONE) {
            periodic.XboxLeftY = 0.0;
        } else {
            periodic.XboxLeftY = LeftY;
        }
        if(Math.abs(RightX) < Constants.Joysticks.XBOX_DEADZONE) {
            periodic.XboxRightX = 0.0;
        } else {
            periodic.XboxRightX = RightX;
        }
    }

    @Override
    public void writePeriodicOutputs() {
        if (periodic.state != State.ChargeStationLock) {
            frontLeftModule.set(
                    periodic.states[0].speedMetersPerSecond / Constants.DriveTrain.SWERVE_MAX_VELOCITY_METERS_PER_SECOND * Constants.DriveTrain.SWERVE_MAX_VOLTAGE,
                    periodic.states[0].angle.getRadians());
            frontRightModule.set(
                    periodic.states[1].speedMetersPerSecond / Constants.DriveTrain.SWERVE_MAX_VELOCITY_METERS_PER_SECOND * Constants.DriveTrain.SWERVE_MAX_VOLTAGE,
                    periodic.states[1].angle.getRadians());
            backLeftModule.set(
                    periodic.states[2].speedMetersPerSecond / Constants.DriveTrain.SWERVE_MAX_VELOCITY_METERS_PER_SECOND * Constants.DriveTrain.SWERVE_MAX_VOLTAGE,
                    periodic.states[2].angle.getRadians());
            backRightModule.set(
                    periodic.states[3].speedMetersPerSecond / Constants.DriveTrain.SWERVE_MAX_VELOCITY_METERS_PER_SECOND * Constants.DriveTrain.SWERVE_MAX_VOLTAGE,
                    periodic.states[3].angle.getRadians());
        } else {
            frontLeftModule.set(0, 1.184);
            frontRightModule.set(0, 2.401);
            backLeftModule.set(0, 2.350);
            backRightModule.set(0, 0.568);
        }
    }

    @Override
    public void outputTelemetry() {
        Pose2d poseEstimatorPose = periodic.poseEstimator.getEstimatedPosition();
        poseEstimatorPub.set(new double[] {
            poseEstimatorPose.getX(),
            poseEstimatorPose.getY(),
            poseEstimatorPose.getRotation().getRadians()
        });
        // poseEstimatorPub.set(new double[] {
        //     periodic.simPoseEstimator.getX(),
        //     periodic.simPoseEstimator.getY(),
        //     periodic.simPoseEstimator.getRotation().getRadians()
        // });
        swervePub.set(new double[] {
                frontLeftModule.getSteerAngle(), frontLeftModule.getDriveVelocity(),
                frontRightModule.getSteerAngle(), frontRightModule.getDriveVelocity(),
                backLeftModule.getSteerAngle(), backLeftModule.getDriveVelocity(),
                backRightModule.getSteerAngle(), backRightModule.getDriveVelocity(),
        });
        setpointPub.set(new double[] {
                frontLeftModule.getDesiredSteerAngle(), periodic.states[0].speedMetersPerSecond,
                frontRightModule.getDesiredSteerAngle(), periodic.states[1].speedMetersPerSecond,
                backLeftModule.getDesiredSteerAngle(), periodic.states[2].speedMetersPerSecond,
                backRightModule.getDesiredSteerAngle(), periodic.states[3].speedMetersPerSecond,
        });
        encoderPub.set(new double[] {
                frontLeftModule.getDriveEncoder(),
                frontRightModule.getDriveEncoder(),
                backLeftModule.getDriveEncoder(),
                backRightModule.getDriveEncoder(),
        });
        statePub.set(periodic.state.toString());
    }

    @Override
    public void reset() {
        setChassisSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0));
        zeroGyroHeading();
        zeroDriveEncoders();
    }

    public SwerveModulePosition[] getSwerveModulePositions() {
        return new SwerveModulePosition[] {
            new SwerveModulePosition(
                frontLeftModule.getDriveEncoder() / Constants.DriveTrain.DRIVE_ENCODER_TO_METERS,
                Rotation2d.fromRadians(frontLeftModule.getSteerAngle())),
            new SwerveModulePosition(
                frontRightModule.getDriveEncoder() / Constants.DriveTrain.DRIVE_ENCODER_TO_METERS,
                Rotation2d.fromRadians(frontRightModule.getSteerAngle())),
            new SwerveModulePosition(
                backLeftModule.getDriveEncoder() / Constants.DriveTrain.DRIVE_ENCODER_TO_METERS,
                Rotation2d.fromRadians(backLeftModule.getSteerAngle())),
            new SwerveModulePosition(
                backRightModule.getDriveEncoder() / Constants.DriveTrain.DRIVE_ENCODER_TO_METERS,
                Rotation2d.fromRadians(backRightModule.getSteerAngle())),
        };
    }

    public void zeroDriveEncoders() {
        frontLeftModule.resetDriveEncoder();
        frontRightModule.resetDriveEncoder();
        backLeftModule.resetDriveEncoder();
        backRightModule.resetDriveEncoder();
    }

    public void zeroGyroHeading() {
        pigeon.setFusedHeading(0);
    }

    public Rotation2d getGyroRotation() {
        return Rotation2d.fromDegrees(pigeon.getFusedHeading());
    }

    public Rotation2d getPoseEstimatorRotation() {
        return periodic.poseEstimator.getEstimatedPosition().getRotation();
    }

    public void setState(State state) {
        periodic.state = state;
    }

    public void setChassisSpeeds(ChassisSpeeds speeds) {
        periodic.speeds = speeds;
    }

    public Pose2d getPose() {
        return periodic.poseEstimator.getEstimatedPosition();
    }

    public Supplier<Pose2d> getPoseSupplier() {
        Supplier<Pose2d> supplier = () -> periodic.poseEstimator.getEstimatedPosition();
        return supplier;
    }

    public Consumer<ChassisSpeeds> setModuleStates() {
        Consumer<ChassisSpeeds> testSpeeds = (speeds) -> periodic.PPspeeds = speeds;
        return testSpeeds;
    }
}
