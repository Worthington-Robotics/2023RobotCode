package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import java.lang.Math;
import frc.lib.util.HIDHelper;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import com.ctre.phoenix.sensors.PigeonIMU;
import frc.lib.loops.ILooper;
import frc.lib.loops.Loop;
import frc.robot.Constants;
import edu.wpi.first.math.filter.LinearFilter;

public class DriveTrain extends Subsystem {
    private static DriveTrain instance = new DriveTrain();
    public static DriveTrain getInstance() {
        return instance;
    }
    private DriveIO periodic;

    // Drive motors
    private TalonFX forwardRightMotor, rearRightMotor, forwardLeftMotor, rearLeftMotor;
    // Gyroscope
    private PigeonIMU gyro;
    // Solenoid used to change gear
    private DoubleSolenoid transmissionSolenoid;

    // Filters used for open loop drive
    private LinearFilter leftFilter, rightFilter;
    // Filter for drive delta
    public LinearFilter driveDeltaFilter;

    public class DriveIO extends PeriodicIO {
        // Read ticks from the left and right drivetrain encoders
        public double leftEncoderTicks, rightEncoderTicks;
        // Joystick X and Y values
        public double xValue, yValue;
        // Motor demands for left and right sides
        public double leftDemand, rightDemand;
        // Error from desired distance for the left and right encoders
        public double leftError = 0;
        public double rightError = 0;
        public double averageError = 0;
        public double integralError = 0;
        public double forwardDerivativeError = 0;
        // The current facing direction of the robot
        public double heading, rawHeading;
        // The heading we are trying to reach
        public double targetHeading;
        // Error from desired heading
        public double headingError;
        // Desired distance to reach
        public double targetDistance;
        // Axis values from the joystick
        public double[] operatorInput = { 0, 0, 0, 0 };
        // Current drive mode
        public DriveMode currentMode = DriveMode.STOPPED;
        // Amount of heading correction to be made while driving forward
        public double driveHeadingCorrect;
        // Total right encoder ticks
        public double totalRightEncoder;
        // Total left encoder ticks
        public double totalLeftEncoder;
        // Left encoder previous distance
        public double leftEncoderPrevDistance;
        // Right encoder previous distance
        public double rightEncoderPrevDistance;
        // Delta right encoder
        public double gyroTilt;
        // Delta from last gyro tilt, used for D term
        public double tiltDelta;
        public boolean gyroLock = false;
        public boolean driveLevelAccepted = false;
    }

    public DriveTrain() {
        periodic = new DriveIO();
        transmissionSolenoid = new DoubleSolenoid(
            0,
            PneumaticsModuleType.CTREPCM,
            Constants.DRIVE_TRANSMISSION_FORWARD, Constants.DRIVE_TRANSMISSION_REVERSE
        );
        // extraSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
        
        gyro = new PigeonIMU(Constants.PIGEON_ID);

        forwardRightMotor = new TalonFX(Constants.DRIVE_FRONT_RIGHT_ID);
        forwardRightMotor.setNeutralMode(NeutralMode.Brake);
        rearRightMotor = new TalonFX(Constants.DRIVE_BACK_RIGHT_ID);
        rearRightMotor.setNeutralMode(NeutralMode.Brake);
        forwardLeftMotor = new TalonFX(Constants.DRIVE_FRONT_LEFT_ID);
        forwardLeftMotor.setNeutralMode(NeutralMode.Brake);
        rearLeftMotor = new TalonFX(Constants.DRIVE_BACK_LEFT_ID);
        rearLeftMotor.setNeutralMode(NeutralMode.Brake);

        forwardLeftMotor.setInverted(true);
        rearLeftMotor.setInverted(true);

        leftFilter = LinearFilter.singlePoleIIR(Constants.OPEN_LOOP_FILTER, 0.02);
        rightFilter = LinearFilter.singlePoleIIR(Constants.OPEN_LOOP_FILTER, 0.02);
        driveDeltaFilter = LinearFilter.singlePoleIIR(Constants.DRIVE_FORWARD_D_FILT, 0.02);
    }

    public enum DriveMode {
        OPEN_LOOP,
        TURN,
        MOVE_FORWARD,
        STOPPED,
        AUTO_LEVEL
    }

    @Override
    public void readPeriodicInputs() {
        final double lastTilt = periodic.gyroTilt;
        periodic.gyroTilt = gyro.getRoll() * -1.0;
        SmartDashboard.putNumber("Drive/Unfiltered Delta", periodic.gyroTilt - lastTilt);
        // periodic.tiltDelta = tiltDeltaFilter.calculate(periodic.gyroTilt - lastTilt);
        periodic.rawHeading = gyro.getFusedHeading();
        periodic.heading = normalizeHeading(periodic.rawHeading);

        periodic.leftEncoderTicks = forwardLeftMotor.getSelectedSensorPosition();
        periodic.rightEncoderTicks = forwardRightMotor.getSelectedSensorPosition();
        periodic.operatorInput = HIDHelper.getAdjStick(Constants.MASTER_STICK);
        periodic.xValue = periodic.operatorInput[0];
        periodic.yValue = periodic.operatorInput[1];

        // These are important derived values that are also read by actions so they should be updated here
        periodic.headingError = normalizeHeadingError(periodic.targetHeading - periodic.heading);
        double oldError = (periodic.leftError + periodic.rightError) / 2.0;
        periodic.leftError = periodic.targetDistance - periodic.leftEncoderTicks;
        periodic.rightError = periodic.targetDistance - periodic.rightEncoderTicks;
        periodic.averageError = (periodic.leftError + periodic.rightError) / 2.0;
        periodic.forwardDerivativeError = driveDeltaFilter.calculate(periodic.averageError - oldError);
    }

    @Override
    public void writePeriodicOutputs() {
        if(periodic.currentMode != DriveMode.STOPPED) {
            forwardLeftMotor.set(ControlMode.PercentOutput, periodic.leftDemand);
            rearLeftMotor.set(ControlMode.Follower, Constants.DRIVE_FRONT_LEFT_ID);
            forwardRightMotor.set(ControlMode.PercentOutput, periodic.rightDemand);
            rearRightMotor.set(ControlMode.Follower, Constants.DRIVE_FRONT_RIGHT_ID);
        } else {
            forwardLeftMotor.set(ControlMode.Disabled, 0);
            rearLeftMotor.set(ControlMode.Disabled, 0);
            forwardRightMotor.set(ControlMode.Disabled, 0);
            rearRightMotor.set(ControlMode.Disabled, 0);
        }
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Drive/Joystick X", periodic.xValue);
        SmartDashboard.putNumber("Drive/Joystick Y", periodic.yValue);
        SmartDashboard.putNumber("Drive/Right Encoder", periodic.rightEncoderTicks);
        SmartDashboard.putNumber("Drive/Left Encoder", periodic.leftEncoderTicks);
        SmartDashboard.putNumber("Drive/Right Demand", periodic.rightDemand);
        SmartDashboard.putNumber("Drive/Left Demand", periodic.leftDemand);
        SmartDashboard.putNumber("Drive/Right Error", periodic.rightError);
        SmartDashboard.putNumber("Drive/Left Error", periodic.leftError);
        SmartDashboard.putNumber("Drive/Target Distance", periodic.targetDistance);
        SmartDashboard.putString("Drive/Mode", periodic.currentMode.toString());
        SmartDashboard.putNumber("Drive/Average Encoder Error", getEncoderError());
        SmartDashboard.putNumber("Drive/Normalized Heading", periodic.heading);
        SmartDashboard.putNumber("Drive/Raw Heading", periodic.rawHeading);
        SmartDashboard.putNumber("Drive/Heading Error", periodic.headingError);
        SmartDashboard.putNumber("Drive/Target Heading", periodic.targetHeading);
        SmartDashboard.putNumber("Drive/Heading Correction", periodic.driveHeadingCorrect);
        SmartDashboard.putNumber("Drive/Total Left Encoder", periodic.totalLeftEncoder);
        SmartDashboard.putNumber("Drive/Total Right Encoder", periodic.totalRightEncoder);
        SmartDashboard.putNumber("Drive/Pitch", periodic.gyroTilt);
        SmartDashboard.putNumber("Drive/Heading", (-periodic.rawHeading + 360) % 360);
        SmartDashboard.putNumber("Drive/Tilt Delta", periodic.tiltDelta);
    }

    @Override
    public void reset() {
        resetEncoders();
        periodic.rightError = 0;
        periodic.leftError = 0;
        periodic.targetDistance = 0;
        periodic.leftDemand = 0;
        periodic.rightDemand = 0;

        periodic.currentMode = DriveMode.OPEN_LOOP;
        periodic.gyroLock = false;

        transmissionSolenoid.set(Value.kReverse);
        gyro.setFusedHeading(0);
    }

    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                periodic.leftEncoderPrevDistance = forwardLeftMotor.getSelectedSensorPosition();
                periodic.rightEncoderPrevDistance = forwardRightMotor.getSelectedSensorPosition();
            }

            @Override
            public void onLoop(double timestamp) {
                switch (periodic.currentMode) {
                    case OPEN_LOOP:
                        openLoop();
                        periodic.driveLevelAccepted = false;
                        break;
                    case TURN:
                        turn();
                        periodic.driveLevelAccepted = false;
                        break;
                    case MOVE_FORWARD:
                        moveForward();
                        periodic.driveLevelAccepted = false;
                        break;
                    case STOPPED:
                        periodic.leftDemand = 0;
                        periodic.rightDemand = 0;
                        periodic.driveLevelAccepted = false;
                        break;
                    case AUTO_LEVEL:
                        autoLevel();
                        if(getLevelError() < 1.0){
                            periodic.driveLevelAccepted = true;
                        } else {
                            periodic.driveLevelAccepted = false;
                        }
                        break;
                }
            }

            @Override
            public void onStop(double timestamp) {
                reset();
            }
        });
    }

    public void resetEncoders() {
        periodic.totalLeftEncoder += periodic.leftEncoderTicks;
        periodic.totalRightEncoder += periodic.rightEncoderTicks;
        forwardLeftMotor.setSelectedSensorPosition(0);
        forwardRightMotor.setSelectedSensorPosition(0);
        rearLeftMotor.setSelectedSensorPosition(0);
        rearRightMotor.setSelectedSensorPosition(0);
    }

    public void setDesiredHeading(double theta) {
        periodic.targetHeading = theta;
        periodic.headingError = normalizeHeadingError(normalizeHeading(periodic.rawHeading) - theta);
    }

    public void setTargetDistance(double distance) {
        resetEncoders();
        periodic.targetDistance = distance;
        periodic.leftError = distance;
        periodic.rightError = distance;
    }

    public void setTurning(double theta) {
        periodic.currentMode = DriveMode.TURN;
        setDesiredHeading(theta);
    }

    public void setHighGear() {
        transmissionSolenoid.set(Value.kForward);
    }

    public void setLowGear() {
        transmissionSolenoid.set(Value.kReverse);
    }

    public void setGyroLock(boolean locked) {
        periodic.gyroLock = locked;
    }

    public double getLeftEncoderDistance() {
        return periodic.totalLeftEncoder + periodic.leftEncoderTicks;
    }

    public double getRawHeading() {
        return periodic.rawHeading;
    }

    public double getNormalizedHeading() {
        return periodic.heading;
    }

    public double getHeadingDegrees() {
        return periodic.heading;
    }

    public double getRightEncoderDistance() {
        return periodic.totalRightEncoder + periodic.rightEncoderTicks;
    }

    public boolean getDriveLevelAccepted() {
        return periodic.driveLevelAccepted;
    }

    public double getEncoderTicks() {
        return (periodic.leftEncoderTicks + periodic.rightEncoderTicks) / 2.0;
    }

    public double getTargetDistance() {
        return periodic.targetDistance;
    }

    public void setOpenLoop() {
        periodic.currentMode = DriveMode.OPEN_LOOP;
    }

    public void setGyro(double heading) {
        gyro.setFusedHeading(heading);
    }

    public void setMoveForward(double distance) {
        periodic.currentMode = DriveMode.MOVE_FORWARD;
        setTargetDistance(distance);
    }

    public void setAutoLevel() {
        periodic.currentMode = DriveMode.AUTO_LEVEL;
    }

    public void setStopped() {
        periodic.currentMode = DriveMode.STOPPED;
    }

    private void turn() {
        periodic.leftDemand = periodic.headingError * Constants.ANGLE_KP;
        periodic.rightDemand = periodic.headingError * - Constants.ANGLE_KP;

        // Normalize power
        periodic.leftDemand = clampDriveSpeed(periodic.leftDemand,
            Constants.DRIVE_TURN_MINIMUM_SPEED, Constants.DRIVE_TURN_MAXIMUM_SPEED);
        periodic.rightDemand = clampDriveSpeed(periodic.rightDemand,
            Constants.DRIVE_TURN_MINIMUM_SPEED, Constants.DRIVE_TURN_MAXIMUM_SPEED);
    }

    private void moveForward() {
        periodic.driveHeadingCorrect = 0.0;
        periodic.leftDemand = periodic.leftError * Constants.DRIVE_FORWARD_KP
            + periodic.forwardDerivativeError * Constants.DRIVE_FORWARD_KD;
        periodic.rightDemand = periodic.rightError * Constants.DRIVE_FORWARD_KP
            + periodic.forwardDerivativeError * Constants.DRIVE_FORWARD_KD;
        
        // Normalize power
        periodic.leftDemand = clampDriveSpeed(periodic.leftDemand, 
            Constants.DRIVE_FORWARD_MINIMUM_SPEED, Constants.DRIVE_FORWARD_MAXIMUM_SPEED);
        periodic.rightDemand = clampDriveSpeed(periodic.rightDemand, 
            Constants.DRIVE_FORWARD_MINIMUM_SPEED, Constants.DRIVE_FORWARD_MAXIMUM_SPEED);

        // Correct for heading error
        periodic.driveHeadingCorrect = periodic.headingError * Constants.DRIVE_FORWARD_HEADING_KP;
        periodic.leftDemand += periodic.driveHeadingCorrect;
        periodic.rightDemand -= periodic.driveHeadingCorrect;

        // Final clamp put in as a safety check
        periodic.leftDemand = clampDriveSpeed(periodic.leftDemand, 0.0, Constants.DRIVE_FORWARD_MAXIMUM_SPEED);
        periodic.rightDemand = clampDriveSpeed(periodic.rightDemand, 0.0, Constants.DRIVE_FORWARD_MAXIMUM_SPEED);
    }

    public double getHeadingError() {
        return periodic.headingError;
    }

    // Gets average error from the encoders
    public double getEncoderError() {
        return (periodic.rightError + periodic.leftError) / 2.0;
    }

    // Sets motor demands in open loop
    private void openLoop() {
        periodic.leftDemand = periodic.yValue;
        periodic.rightDemand = periodic.yValue;
        if (periodic.gyroLock) {
            periodic.driveHeadingCorrect = periodic.headingError * Constants.DRIVE_FORWARD_HEADING_KP;
            periodic.leftDemand += periodic.driveHeadingCorrect;
            periodic.rightDemand -= periodic.driveHeadingCorrect;
        } else {
            periodic.leftDemand += periodic.xValue;
            periodic.rightDemand -= periodic.xValue; 
        }

        // Normalize 
        periodic.leftDemand = clampDriveSpeed(periodic.leftDemand, 0.0, 1.0);
        periodic.rightDemand = clampDriveSpeed(periodic.rightDemand, 0.0, 1.0);

        // Filter output
        periodic.leftDemand = leftFilter.calculate(periodic.leftDemand);
        periodic.rightDemand = rightFilter.calculate(periodic.rightDemand);
    }

    // Auto-leveling mode for charge station
    private void autoLevel() {
        final double levelError = Constants.DRIVE_LEVEL_ZERO + periodic.gyroTilt;
        final double power = (levelError * Constants.DRIVE_LEVEL_KP);
        double minPower = 0;

        periodic.leftDemand = - power;
        periodic.rightDemand = - power;

        if(periodic.gyroLock) {
            periodic.driveHeadingCorrect = periodic.headingError * Constants.DRIVE_FORWARD_HEADING_KP;
            periodic.leftDemand += periodic.driveHeadingCorrect;
            periodic.rightDemand -= periodic.driveHeadingCorrect;
        }

        if(Math.abs(levelError) > 3.0){
            minPower = 0.07;
        }
        // Normalize power
        if(Math.abs(levelError) > 3.0){
            minPower = 0.07;
        }
        periodic.leftDemand = clampDriveSpeed(periodic.leftDemand,
            minPower, Constants.DRIVE_LEVEL_MAX_SPEED);
        periodic.rightDemand = clampDriveSpeed(periodic.rightDemand,
            minPower, Constants.DRIVE_LEVEL_MAX_SPEED);
    }

    public double getLevelError() {
        return Constants.DRIVE_LEVEL_ZERO + periodic.gyroTilt;
    }

    public static double clampDriveSpeed(double demand, double min, double max) {
        if (Math.abs(demand) < min) {
            return Math.signum(demand) * min;
        }
        if (Math.abs(demand) > max) {
            return Math.signum(demand) * max;
        } else {
            return demand;
        }
    }

    // Normalizes heading direction
    public static double normalizeHeading(double heading) {
        final double wrappedHeading = heading % 360;
        if (wrappedHeading > 180.0) {
            return wrappedHeading - 360.0;
        }
        if (wrappedHeading < -180.0) {
            return wrappedHeading + 360.0;
        } else {
            return wrappedHeading;
        }
    }

    // Normalizes heading error
    public static double normalizeHeadingError(double error) {
        if (Math.abs(error) > 180) {
            if (Math.signum(error) == 1.0) {
                return error - 360.0;
            } else {
                return error + 360.0;
            }
        } else {
            return error;
        }
    }

    // Gets a target heading which is the closest between 0 and 180
    public static double getAlignedTargetHeading(double currentHeading) {
        final double normalized = normalizeHeading(currentHeading);
        if (Math.abs(normalized) < 90) {
            return 0 * Math.signum(normalized);
        } else {
            return 180 * Math.signum(normalized);
        }
    }

    public LogData getLogger() {
		return periodic;
	}
}
