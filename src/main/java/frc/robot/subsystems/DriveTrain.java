package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import java.lang.Math;
import java.util.Currency;

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
        public double forwardDerivativeError = 0;
        // The current facing direction of the robot
        public double normalizedHeading, rawHeading;
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
        public double gyroTilt;
        // Delta from last gyro tilt, used for D term
        public double tiltDelta;
        //declare whether it is gyro locked or not
        public boolean gyroLock = false;
        //when a raw input power is put in the mtoros
        public double inputDrivePower;
        //for the autolevel to determine the robot orientation
        public boolean moveForward = true;
    }

    public DriveTrain() {
        periodic = new DriveIO();
        transmissionSolenoid = new DoubleSolenoid(
            0,
            PneumaticsModuleType.CTREPCM,
            Constants.DRIVE_TRANSMISSION_FORWARD, Constants.DRIVE_TRANSMISSION_REVERSE
        );
        
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

        
        reset();

        leftFilter = LinearFilter.singlePoleIIR(Constants.OPEN_LOOP_FILTER, 0.02);
        rightFilter = LinearFilter.singlePoleIIR(Constants.OPEN_LOOP_FILTER, 0.02);
        driveDeltaFilter = LinearFilter.singlePoleIIR(Constants.DRIVE_FORWARD_D_FILT, 0.02);
    }

    public enum DriveMode {
        OPEN_LOOP,
        TURN,
        MOVE_FORWARD,
        STOPPED,
        AUTO_LEVEL,
        FOLLOW_MOTOR_POWER
    }

    @Override
    public void readPeriodicInputs() {
        final double lastTilt = periodic.gyroTilt;
        periodic.gyroTilt = gyro.getRoll() * -1.0;
        periodic.tiltDelta = periodic.gyroTilt - lastTilt;
        //periodic.tiltDelta = tiltDeltaFilter.calculate(periodic.gyroTilt - lastTilt);
        periodic.rawHeading = gyro.getFusedHeading();
        periodic.normalizedHeading = normalizeHeading(periodic.rawHeading);

        periodic.leftEncoderTicks = forwardLeftMotor.getSelectedSensorPosition();
        periodic.rightEncoderTicks = forwardRightMotor.getSelectedSensorPosition();
        periodic.operatorInput = HIDHelper.getAdjStick(Constants.MASTER_STICK);
        periodic.xValue = periodic.operatorInput[0] * (0.7);
        periodic.yValue = periodic.operatorInput[1];

        // These are important derived values that are also read by actions so they should be updated here
        periodic.headingError = normalizeHeadingError(periodic.targetHeading - periodic.normalizedHeading);
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
        SmartDashboard.putNumber("Drive/Raw Heading", periodic.rawHeading);
        SmartDashboard.putNumber("Drive/Right Demand", periodic.rightDemand);
        SmartDashboard.putNumber("Drive/Left Demand", periodic.leftDemand);
        SmartDashboard.putNumber("Drive/Right Error", periodic.rightError);
        SmartDashboard.putNumber("Drive/Left Error", periodic.leftError);
        SmartDashboard.putNumber("Drive/Target Distance", periodic.targetDistance);
        SmartDashboard.putString("Drive/Mode", periodic.currentMode.toString());
        SmartDashboard.putNumber("Drive/Average Encoder Error", getEncoderError());
        SmartDashboard.putNumber("Drive/Normalized Heading", periodic.normalizedHeading);
        SmartDashboard.putNumber("Drive/Heading Error", periodic.headingError);
        SmartDashboard.putNumber("Drive/Target Heading", periodic.targetHeading);
        SmartDashboard.putNumber("Drive/Heading Correction", periodic.driveHeadingCorrect);
        SmartDashboard.putNumber("Drive/Pitch", periodic.gyroTilt);
        SmartDashboard.putNumber("Drive/Heading", (-periodic.rawHeading + 360) % 360);
        SmartDashboard.putNumber("Drive/Tilt Delta", periodic.tiltDelta);
        SmartDashboard.putBoolean("Drive/Moving Forward", periodic.moveForward);
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
            }

            @Override
            public void onLoop(double timestamp) {
                switch (periodic.currentMode) {
                    case OPEN_LOOP:
                       // setGyroLock(true);
                        openLoop();
                        break;
                    case TURN:
                        turn();
                        break;
                    case MOVE_FORWARD:
                        moveForward();
                        break;
                    case STOPPED:
                        periodic.leftDemand = 0;
                        periodic.rightDemand = 0;
                        break;
                    case AUTO_LEVEL:
                        autoLevel(periodic.moveForward);
                        break;
                    case FOLLOW_MOTOR_POWER:
                        periodic.leftDemand = periodic.inputDrivePower;
                        periodic.rightDemand = periodic.inputDrivePower;
                        lockGyro();
                        break;
                }
            }

            @Override
            public void onStop(double timestamp) {
            }
        });
    }

    public void resetEncoders() {
        forwardLeftMotor.setSelectedSensorPosition(0);
        forwardRightMotor.setSelectedSensorPosition(0);
    }

    public void setOpenLoop() {
        periodic.currentMode = DriveMode.OPEN_LOOP;
    }

    public void setMoveForward(double distance) {
        periodic.currentMode = DriveMode.MOVE_FORWARD;
        setTargetDistance(distance);
    }

    public void setAutoLevel(boolean moveForward) {
        periodic.currentMode = DriveMode.AUTO_LEVEL;
        periodic.moveForward = moveForward;
    }

    public void setStopped() {
        periodic.currentMode = DriveMode.STOPPED;
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

    
    public void setGyro(double heading) {
        gyro.setFusedHeading(heading);
    }

    public double getRawHeading() {
        return periodic.rawHeading;
    }

    public double getNormalizedHeading() {
        return periodic.normalizedHeading;
    }

    public double getEncoderTicks() {
        return (periodic.leftEncoderTicks + periodic.rightEncoderTicks) / 2.0;
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
        lockGyro();

        // Final clamp put in as a safety check
        periodic.leftDemand = clampDriveSpeed(periodic.leftDemand, 0.0, Constants.DRIVE_FORWARD_MAXIMUM_SPEED);
        periodic.rightDemand = clampDriveSpeed(periodic.rightDemand, 0.0, Constants.DRIVE_FORWARD_MAXIMUM_SPEED);
    }

    public void setMotorPower(double motorPower){
        periodic.currentMode = DriveMode.FOLLOW_MOTOR_POWER;
        periodic.inputDrivePower = motorPower;
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
           lockGyro();
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
    private void autoLevel(boolean isForward) {
        double levelError = Constants.DRIVE_LEVEL_ZERO + periodic.gyroTilt;
        double power;
        if(levelError > 9){
            power = 0.1;
            if(!isForward){
                power *= -1;
            }
        } else if (levelError < -9){
            power = -0.1;
            if(!isForward){
                power *= -1;
            }
        } else {
            power = 0;
        }

        if(periodic.gyroLock) {
            lockGyro();
        }


        periodic.rightDemand = power;
        periodic.leftDemand = power;
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

    public void lockGyro() {
        periodic.driveHeadingCorrect = periodic.headingError * Constants.DRIVE_FORWARD_HEADING_KP;
        periodic.leftDemand += periodic.driveHeadingCorrect;
        periodic.rightDemand -= periodic.driveHeadingCorrect;
    }

    public LogData getLogger() {
		return periodic;
	}

    public boolean getDeltaPitchAccepted(boolean moveForward) {
        if(moveForward) { // Verify that battery above horizon is positive tilt
            periodic.tiltDelta *= -1.0;
        }

        if(periodic.tiltDelta < 0.0) { // return periodic.tiltDelta < 0.0 // delta should change based on pigin accuracy
            return true;
        }
        else {
            return false;
        }
    }
}
