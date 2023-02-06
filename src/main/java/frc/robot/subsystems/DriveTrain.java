package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.drivers.PIDF;
import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Rotation2d;
import frc.lib.geometry.Twist2d;
import frc.lib.loops.ILooper;
import frc.lib.loops.Loop;
import frc.lib.models.DriveMotionPlanner;
import frc.lib.trajectory.*;
import frc.lib.util.DriveSignal;
import frc.lib.util.HIDHelper;
import frc.robot.Constants;



public class DriveTrain extends Subsystem {

    // construct one and only 1 instance of this class
    private static DriveTrain m_DriveInstance = new DriveTrain();

    public static DriveTrain getInstance() {
        return m_DriveInstance;
    }

    public PeriodicIO getLogger() {
        return periodic;
    }

    // used internally for data
    private DriveControlState mDriveControlState = DriveControlState.OPEN_LOOP;
    private boolean mOverrideTrajectory = false;
    //private DriveMotionPlanner mMotionPlanner;
    private DriveIO periodic;
    private PigeonIMU pigeonIMU;
    private DoubleSolenoid transmission;
    private TalonFX driveFrontLeft, driveBackRight, driveFrontRight, driveBackLeft;
    //private PIDF anglePID;

    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                synchronized (DriveTrain.this) {
                }

            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (DriveTrain.this) {
                    if (Constants.ENABLE_MP_TEST_MODE && DriverStation.isTest()) {
                        mDriveControlState = DriveControlState.PROFILING_TEST;
                    }
                    if (periodic.inverse) {
                        periodic.operatorInput[1] *= -1;
                    }
                    switch (mDriveControlState) {
                    // case PATH_FOLLOWING:
                    //     updatePathFollower();
                    //     break;
                    case PROFILING_TEST:
                        if (Constants.RAMPUP) {
                            periodic.leftDemand = periodic.rampUpCounter * .0025 + .01;
                            periodic.rightDemand = periodic.rampUpCounter * .0025 + .01;
                            periodic.rampUpCounter++;
                        } else if (DriverStation.isTest()) {
                            periodic.leftDemand = radiansPerSecondToTicksPer100ms(
                                    metersToRadians(Constants.MP_TEST_SPEED));
                            periodic.rightDemand = radiansPerSecondToTicksPer100ms(
                                    metersToRadians(Constants.MP_TEST_SPEED));
                        }
                        break;
                    case OPEN_LOOP:
                        if (!DriverStation.isAutonomous()) {
                            setOpenLoop(arcadeDrive(periodic.operatorInput[1], periodic.operatorInput[0]));
                            automaticShifter();
                        }
                        break;
                    // case ANGLE_PID:
                    //     periodic.PIDOutput = anglePID.update(periodic.gyroHeading.getDegrees());
                    //     DriveSignal angleSignal = arcadeDrive(periodic.operatorInput[1], periodic.PIDOutput);
                    //     periodic.rightDemand = angleSignal.getRight();
                    //     periodic.leftDemand = angleSignal.getLeft();
                    //     automaticShifter();
                    //     break;
                    // case TURN_LOCKOUT:
                    // //turn radius of .7
                    //     Twist2d radius;
                    //     if(periodic.turnLockoutCCW)
                    //     {
                    //     radius = new Twist2d(periodic.turnLockoutRadius, periodic.turnLockoutRadius, 90);
                    //     }
                    //     else
                    //     {
                    //     radius = new Twist2d(periodic.turnLockoutRadius, -periodic.turnLockoutRadius, -90);
                    //     }
                    //     radius.scaled(periodic.operatorInput[1] * .25);
                    //     DriveSignal drivesignal = Kinematics.inverseKinematics(radius);
                    //     periodic.rightDemand = drivesignal.getRight();
                    //     periodic.leftDemand = drivesignal.getLeft();
                    //     automaticShifter();
                    //     break;
                    default:
                        System.out.println("You fool, unexpected control state");

                    }
                    if (Constants.DEBUG) {
                        // if (periodic.savePIDSettings) {
                        //     System.out.print("Configed PID");
                        //     configPID();
                        // }
                    }
                }
            }

            @Override
            public void onStop(double timestamp) {

            }
        });
    }

    @Override
    public synchronized void readPeriodicInputs() {
        if (SmartDashboard.getBoolean("Drive/PID/SaveChanges", false) && Constants.DEBUG) {
            updateDrivePID(SmartDashboard.getNumber("Drive/PID/P", 0), SmartDashboard.getNumber("Drive/PID/I", 0),
                    SmartDashboard.getNumber("Drive/PID/D", 0), SmartDashboard.getNumber("Drive/PID//F", 0));
        }
        if(!Constants.WHEELS)
        {
            if (periodic.TransState) {
                periodic.operatorInput = HIDHelper.getAdjStick(Constants.MASTER_STICK_SHIFTED);
            } else {
                periodic.operatorInput = HIDHelper.getAdjStick(Constants.MASTER_STICK);
            }
        } else {
        periodic.operatorInput = HIDHelper.getAdjStick(Constants.WHEEL_STICK);
        periodic.operatorInput[1] = HIDHelper.getAxisMapped(periodic.operatorInput[1], 1, 0);
        periodic.operatorInput[2] = HIDHelper.getAxisMapped(periodic.operatorInput[2], 1, 0);
        periodic.operatorInput[3] = HIDHelper.getAxisMapped(periodic.operatorInput[3], 1, 0);
        }

       // periodic.AnglePIDError = anglePID.getError();
        periodic.gyroHeading = Rotation2d.fromDegrees(pigeonIMU.getFusedHeading()).rotateBy(periodic.gyroOffset);
        periodic.leftVelocityTicksPer100ms = driveFrontLeft.getSelectedSensorVelocity();
        periodic.rightVelocityTicksPer100ms = driveFrontRight.getSelectedSensorVelocity();
        periodic.leftPosTicks = driveFrontLeft.getSelectedSensorPosition();
        periodic.rightPosTicks = driveFrontRight.getSelectedSensorPosition();
        periodic.rightCurrent = driveFrontRight.getSupplyCurrent();
        periodic.leftCurrent = driveFrontLeft.getSupplyCurrent();

    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (mDriveControlState == DriveControlState.OPEN_LOOP || mDriveControlState == DriveControlState.ANGLE_PID || (mDriveControlState == DriveControlState.PROFILING_TEST && Constants.RAMPUP)) {
            driveFrontLeft.set(ControlMode.PercentOutput, periodic.leftDemand);
            driveFrontRight.set(ControlMode.PercentOutput, periodic.rightDemand);
        } else {
            driveFrontLeft.set(ControlMode.Velocity, periodic.leftDemand, DemandType.ArbitraryFeedForward, (periodic.leftFeedforward + Constants.DRIVE_LEFT_KD * periodic.leftAccl / 1023.0));
            driveFrontRight.set(ControlMode.Velocity, periodic.rightDemand, DemandType.ArbitraryFeedForward, (periodic.rightFeedforward + Constants.DRIVE_RIGHT_KD * periodic.rightAccl / 1023.0));
        }
        transmission.set(periodic.TransState ? Value.kForward : Value.kReverse);
    }

    private DriveTrain() {
        SmartDashboard.putBoolean("Drive/PID/SaveChanges", false);
        SmartDashboard.putNumber("Drive/PID/P", 0);
        SmartDashboard.putNumber("Drive/PID/I", 0);
        SmartDashboard.putNumber("Drive/PID/D", 0);
        SmartDashboard.putNumber("Drive/PID/F", 0);
        // anglePID = new PIDF(Constants.ANGLE_KP, Constants.ANGLE_KD);
        // anglePID.setContinuous(true);
        driveFrontLeft = new TalonFX(Constants.DRIVE_FRONT_LEFT_ID);
        driveBackLeft = new TalonFX(Constants.DRIVE_BACK_LEFT_ID);
        driveFrontRight = new TalonFX(Constants.DRIVE_FRONT_RIGHT_ID);
        driveBackRight = new TalonFX(Constants.DRIVE_BACK_RIGHT_ID);
        //pigeonIMU = new PigeonIMU(Constants.PIGION_ID);
        transmission = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.TRANS_LOW_ID, Constants.TRANS_HIGH_ID);
        //mMotionPlanner = new DriveMotionPlanner();
        configTalons();
        reset();

    }
    /**
     * Method that returns whether the robot should shift or not based on the 
     * absolute value of the average linear velocity
     */
    public void automaticShifter()
    {
        periodic.linearVelocity = Math.abs(getLinearVelocity());
        if((periodic.TransState && periodic.linearVelocity < 2.5 || !periodic.TransState && periodic.linearVelocity > 2.25) && !periodic.manualShifterOverride)
        {
            periodic.TransState = !periodic.TransState;
        }
    }

    public void manualShifterOverride(boolean override)
    {
        periodic.manualShifterOverride = override;
    }

    // public PIDF getAnglePID() {
    //     return anglePID;
    // }

    public void setTrans(boolean state) {
        periodic.TransState = state;
    }

    public synchronized Rotation2d getHeading() {
        return periodic.gyroHeading;
    }

    public synchronized void setHeading(Rotation2d heading) {
        System.out.println("SET HEADING: " + heading.getDegrees());
        periodic.gyroOffset = heading.rotateBy(Rotation2d.fromDegrees(pigeonIMU.getFusedHeading()).inverse());
        System.out.println("Gyro offset: " + periodic.gyroOffset.getDegrees());
        periodic.gyroHeading = heading;
    }

    public void updateDrivePID(double P, double I, double D, double F) {
        driveFrontLeft.config_kP(0, P);
        driveFrontLeft.config_kI(0, I);
        driveFrontLeft.config_kD(0, D);
        driveFrontLeft.config_kF(0, F);

        driveFrontRight.config_kP(0, P);
        driveFrontRight.config_kI(0, I);
        driveFrontRight.config_kD(0, D);
        driveFrontRight.config_kF(0, F);
    }

    public double getLeftEncoderRotations() {
        return periodic.leftPosTicks / Constants.DRIVE_ENCODER_PPR;
    }

    public double getRightEncoderRotations() {
        return periodic.rightPosTicks / Constants.DRIVE_ENCODER_PPR;
    }

    public double getLeftEncoderDistance() {
        return rotationsToMeters(getLeftEncoderRotations());
    }

    public double getRightEncoderDistance() {
        return rotationsToMeters(getRightEncoderRotations());
    }

    public double getLeftVelocityNativeUnits() {
        return periodic.leftVelocityTicksPer100ms;
    }

    public double getRightVelocityNativeUnits() {
        return periodic.rightVelocityTicksPer100ms;
    }

    public double getTemperature()
    {
        return driveFrontLeft.getTemperature();
    }

    public double getLeftLinearVelocity() {
        double leftLinearVelocity = rotationsToMeters(getLeftVelocityNativeUnits() * 10.0);
        if(!periodic.TransState)
        {
            leftLinearVelocity /= Constants.DRIVE_ENCODER_PPR;
        }
        else
        {
            leftLinearVelocity /= Constants.DRIVE_ENCODER_PPR_HIGH_GEAR;
        }
        return leftLinearVelocity;
    }

    public double getRightLinearVelocity() {
        double rightLinearVelocity = rotationsToMeters(getRightVelocityNativeUnits() * 10.0);
        if(!periodic.TransState)
        {
            rightLinearVelocity /= Constants.DRIVE_ENCODER_PPR;
        }
        else
        {
            rightLinearVelocity /= Constants.DRIVE_ENCODER_PPR_HIGH_GEAR;
        }
        return rightLinearVelocity;
    }

    public double getLinearVelocity()
    {
        return (getLeftLinearVelocity() + getRightLinearVelocity()) / 2;
    }

    public void reset() {
        // mMotionPlanner.reset();
        // mMotionPlanner.setFollowerType(DriveMotionPlanner.FollowerType.NONLINEAR_FEEDBACK);
        mOverrideTrajectory = false;
        periodic = new DriveIO();
        setHeading(Rotation2d.fromDegrees(0));
        resetEncoders();
    }

    private void resetEncoders() {
        driveFrontRight.setSelectedSensorPosition(0, 0, 0);
        driveFrontLeft.setSelectedSensorPosition(0, 0, 0);
    }

    // private void configPID() {
    //     anglePID.setPID(periodic.PIDPUpdate, 0, periodic.PIDDUpdate);
    // }

    private void configTalons() {
        // primary closed-loop, 100 ms timeout
        ErrorCode sensorPresent = driveFrontLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 100);
        if (sensorPresent != ErrorCode.OK) {
            DriverStation.reportError("Could not detect left encoder: " + sensorPresent, false);
        }
        // DO NOT FORGET THIS! use 5ms packet time on feedback
        driveFrontLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 100);
        driveFrontLeft.selectProfileSlot(0, 0);
        driveFrontLeft.config_kF(0, Constants.DRIVE_LEFT_KF, 0);
        driveFrontLeft.config_kP(0, Constants.DRIVE_LEFT_KP, 0);
        driveFrontLeft.config_kI(0, Constants.DRIVE_LEFT_KI, 0);
        driveFrontLeft.config_kD(0, Constants.DRIVE_LEFT_KD, 0);
        driveFrontLeft.config_IntegralZone(0, 300);
        driveFrontLeft.setInverted(true);
        driveFrontLeft.setNeutralMode(NeutralMode.Brake);
        driveFrontLeft.configVoltageCompSaturation(Constants.DRIVE_VCOMP);
        driveFrontLeft.enableVoltageCompensation(true);
        driveFrontLeft.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 0, 0.02));
        driveFrontLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        driveBackLeft.setInverted(true);
        driveBackLeft.setNeutralMode(NeutralMode.Brake);
        driveBackLeft.configVoltageCompSaturation(Constants.DRIVE_VCOMP);
        driveBackLeft.enableVoltageCompensation(true);
        driveBackLeft.follow(driveFrontLeft);
        driveBackLeft.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 0, 0.02));
        driveBackLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255, 0);
        driveBackLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255, 0);

        // primary closed-loop, 100ms timeout
        sensorPresent = driveFrontRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 100);
        if (sensorPresent != ErrorCode.OK) {
            DriverStation.reportError("Could not detect right encoder: " + sensorPresent, false);
        }
        // DO NOT FORGET THIS! use 5ms packet time on feedback
        driveFrontLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 100);
        driveFrontRight.selectProfileSlot(0, 0);
        driveFrontRight.config_kF(0, Constants.DRIVE_RIGHT_KF, 0);
        driveFrontRight.config_kP(0, Constants.DRIVE_RIGHT_KP, 0);
        driveFrontRight.config_kI(0, Constants.DRIVE_RIGHT_KI, 0);
        driveFrontRight.config_kD(0, Constants.DRIVE_RIGHT_KD, 0);
        driveFrontRight.config_IntegralZone(0, 300);
        driveFrontRight.setInverted(false);
        driveFrontRight.setNeutralMode(NeutralMode.Brake);
        driveFrontRight.configVoltageCompSaturation(Constants.DRIVE_VCOMP);
        driveFrontRight.enableVoltageCompensation(true);
        driveFrontRight.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 0, 0.02));
        driveFrontRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        driveBackRight.setInverted(false);
        driveBackRight.setNeutralMode(NeutralMode.Brake);
        driveBackRight.configVoltageCompSaturation(Constants.DRIVE_VCOMP);
        driveBackRight.enableVoltageCompensation(true);
        driveBackRight.follow(driveFrontRight);
        driveBackRight.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 0, 0.02));
        driveBackRight.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255, 0);
        driveBackRight.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255, 0);

    }

    public void overrideTrajectory(boolean value) {
        mOverrideTrajectory = value;
    }

    // private void updatePathFollower() {
    //     if (mDriveControlState == DriveControlState.PATH_FOLLOWING) {
    //         final double now = Timer.getFPGATimestamp();

    //         DriveMotionPlanner.Output output = mMotionPlanner.update(now, PoseEstimator.getInstance().getFieldToVehicle(now));

    //         periodic.error = mMotionPlanner.error();
    //         periodic.pathSetpoint = mMotionPlanner.setpoint();

    //         if (!mOverrideTrajectory) {
    //             DriveSignal signal = new DriveSignal(radiansPerSecondToTicksPer100ms(output.left_velocity),
    //                     radiansPerSecondToTicksPer100ms(output.right_velocity));
    //             setVelocity(signal, new DriveSignal(output.left_feedforward_voltage / 10, output.right_feedforward_voltage / 10));
    //             periodic.leftAccl = radiansPerSecondToTicksPer100ms(output.left_accel) / 1000; 
    //             periodic.rightAccl = radiansPerSecondToTicksPer100ms(output.right_accel) / 1000; 

    //         } else {
    //             setVelocity(DriveSignal.BRAKE, DriveSignal.BRAKE);
    //             mDriveControlState = DriveControlState.OPEN_LOOP;
    //             mMotionPlanner.reset();

    //         }
    //     } else {
    //         DriverStation.reportError("Drive is not in path following state", false);
    //     }
    // }

    /**
     * Configure talons for open loop control
     *
     * @param signal input to drive train
     */
    public synchronized void setOpenLoop(DriveSignal signal) {
        if (mDriveControlState != DriveControlState.OPEN_LOOP) {
            System.out.println("Switching to open loop");
            driveFrontLeft.set(ControlMode.PercentOutput, 0);
            driveFrontRight.set(ControlMode.PercentOutput, 0);
            mDriveControlState = DriveControlState.OPEN_LOOP;
        }
        periodic.leftDemand = signal.getLeft();
        periodic.rightDemand = signal.getRight();
    }

    public synchronized void setTurnLockout(boolean ccw, double radius) {
        if (mDriveControlState != DriveControlState.TURN_LOCKOUT) {
            System.out.println("Switching to angle control");
            driveFrontLeft.set(ControlMode.PercentOutput, 0);
            driveFrontRight.set(ControlMode.PercentOutput, 0);
            mDriveControlState = DriveControlState.TURN_LOCKOUT;
        }
    }

    /**
     * Configure for Angle PID control
     */
    public synchronized void setAnglePidLoop(DriveSignal signal, double angle) {
        if (mDriveControlState != DriveControlState.ANGLE_PID) {
            System.out.println("Switching to angle control");
            driveFrontLeft.set(ControlMode.PercentOutput, 0);
            driveFrontRight.set(ControlMode.PercentOutput, 0);
            mDriveControlState = DriveControlState.ANGLE_PID;
        }
       // anglePID.setPoint(angle);
        periodic.gyroPidAngle = angle;
        periodic.leftDemand = signal.getLeft();
        periodic.rightDemand = signal.getRight();
    }

    // public boolean getPIDOnTarget() {
    //     return anglePID.onTarget(Constants.ANGLE_PID_EPISLON);
    // }

    public void setInverse(boolean isInverse) {
        periodic.inverse = isInverse;
    }

    /**
     * Configures talons for velocity control
     */
    public synchronized void setVelocity(DriveSignal signal, DriveSignal feedforward) {
        if (mDriveControlState != DriveControlState.PATH_FOLLOWING) {
            System.out.println("Switching to velocity control");
            driveFrontLeft.set(ControlMode.Velocity, 0);
            driveFrontRight.set(ControlMode.Velocity, 0);
            mDriveControlState = DriveControlState.PATH_FOLLOWING;
        }
        periodic.leftDemand = signal.getLeft();
        periodic.rightDemand = signal.getRight();
    }

    // public synchronized void setTrajectory(Trajectory trajectory) {
    //     if (mMotionPlanner != null) {
    //         mOverrideTrajectory = false;
    //         mMotionPlanner.reset();
    //         mMotionPlanner.setTrajectory(trajectory);
    //         mDriveControlState = DriveControlState.PATH_FOLLOWING;
    //     }
    // }

    // public boolean isDoneWithTrajectory() {
    //     if (mMotionPlanner == null || mDriveControlState != DriveControlState.PATH_FOLLOWING) {
    //         return true;
    //     }
    //     return mMotionPlanner.isDone() || mOverrideTrajectory;
    // }

    public void outputTelemetry() {
       // double[] PIDData = anglePID.getPID();
        SmartDashboard.putBoolean("isInverse", periodic.inverse);

        SmartDashboard.putNumber("Drive/AnglePID/Set Point", periodic.gyroPidAngle);
        SmartDashboard.putNumber("Drive/AnglePID/Error", periodic.AnglePIDError);

        SmartDashboard.putString("Drive/Drive State", mDriveControlState.toString());
        SmartDashboard.putNumberArray("Drive/Stick", periodic.operatorInput);
        SmartDashboard.putNumber("Drive/LinearVelocity", getLinearVelocity());
        SmartDashboard.putBoolean("Drive/Shift", periodic.TransState);
        SmartDashboard.putBoolean("Drive/ShiftOverride", periodic.manualShifterOverride);
        SmartDashboard.putNumber("Drive/Error/X", periodic.error.getTranslation().x());
        SmartDashboard.putNumber("Drive/Error/Y", periodic.error.getTranslation().y());
        SmartDashboard.putNumber("Drive/Error/Theta", periodic.error.getRotation().getDegrees());
        SmartDashboard.putNumber("Drive/Temp", driveFrontLeft.getTemperature());

        SmartDashboard.putNumber("Drive/Left/Current", periodic.leftCurrent);
        SmartDashboard.putNumber("Drive/Left/Demand", periodic.leftDemand);
        SmartDashboard.putNumber("Drive/Left/Talon Velocity", periodic.leftVelocityTicksPer100ms);
        SmartDashboard.putNumber("Drive/Left/Encoder Counts", periodic.leftPosTicks);

        SmartDashboard.putNumber("Drive/Right/Current", periodic.rightCurrent);
        SmartDashboard.putNumber("Drive/Right/Demand", periodic.rightDemand);
        SmartDashboard.putNumber("Drive/Right/Talon Velocity", periodic.rightVelocityTicksPer100ms);
        SmartDashboard.putNumber("Drive/Right/Encoder Counts", periodic.rightPosTicks);

        if (Constants.DEBUG) {
            // SmartDashboard.putNumber("Drive/AnglePID/P", PIDData[0]);
            // SmartDashboard.putNumber("Drive/AnglePID/D", PIDData[2]);
        }
    }

    enum DriveControlState {
        OPEN_LOOP, PATH_FOLLOWING, PROFILING_TEST, GYRO_LOCK, ANGLE_PID, TURN_LOCKOUT;

        public String toString() {
            return name().charAt(0) + name().substring(1).toLowerCase();
        }
    }

    public class DriveIO extends PeriodicIO {
        public TimedState pathSetpoint = new TimedState();
        // INPUTS
        public double leftPosTicks = 0;
        public double leftVelocityTicksPer100ms = 0;
        public double leftCurrent = 0;

        public double rightPosTicks = 0;
        public double rightVelocityTicksPer100ms = 0;
        public double rightCurrent = 0;

        public Rotation2d gyroHeading = Rotation2d.identity();
        public Rotation2d gyroOffset = Rotation2d.identity();
        public double gyroPidAngle = 0;
        public double AnglePIDError = 0;

        public Pose2d error = new Pose2d(0, 0, Rotation2d.identity());
        public double[] operatorInput = { 0, 0, 0, 0};
        public boolean inverse = false;
        public double PIDOutput = 0;

        // Smartdashboard Settings
        private double PIDDUpdate = 0;
        private double PIDPUpdate = 0;
        private boolean savePIDSettings = false;

        // OUTPUTS
        public double rampUpCounter = 0;
        public double linearVelocity = 0;
        public boolean TransState = false;
        public boolean manualShifterOverride = false;
        
        public double leftAccl = 0.0;
        public double leftDemand = 0.0;
        public double leftDistance = 0.0;
        public double leftFeedforward = 0.0;

        public double rightAccl = 0.0;
        public double rightDemand = 0.0;
        public double rightDistance = 0.0;
        public double rightFeedforward = 0.0;

        public boolean turnLockoutCCW = false;     
        public double turnLockoutRadius = .7;
    }

    public LogData logData() {
        return periodic;
    }

    /**
     * internal methods beyond this point
     **/

    private static double rotationsToMeters(double rotations) {
        return rotations * Math.PI * Constants.DRIVE_WHEEL_DIAMETER;
    }

    private static double radiansPerSecondToTicksPer100ms(double rad_s) {
        return rad_s / (Math.PI * 2.0) * Constants.DRIVE_ENCODER_PPR / 10.0;
    }

    private static double metersToRadians(double m_sec) {
        return m_sec / (Constants.DRIVE_WHEEL_DIAMETER * Math.PI) * 2 * Math.PI;
    }

    /**
     * Arcade drive method for calculating drivetrain output.
     * <p>
     * defined as positive forward on both outputs and turning right yields positive
     * left output and negative right output
     * 
     * @param xSpeed    desired travel velocity
     * @param zRotation desired rotational velocity
     * @return a drivesignal for open loop use
     */
    private DriveSignal arcadeDrive(double xSpeed, double zRotation) {
        final double maxInput = Math.max(Math.max(Math.abs(xSpeed - zRotation), Math.abs(xSpeed + zRotation)), 1);

        final double rightMotorOutput = (xSpeed + zRotation) / maxInput;
        final double leftMotorOutput = (xSpeed - zRotation) / maxInput;

        return new DriveSignal(rightMotorOutput, leftMotorOutput);
    }

}