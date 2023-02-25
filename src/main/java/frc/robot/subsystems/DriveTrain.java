package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import java.lang.Math;
import frc.lib.util.HIDHelper;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import com.ctre.phoenix.sensors.PigeonIMU;
import frc.lib.loops.ILooper;
import frc.lib.loops.Loop;
import frc.robot.Constants;

public class DriveTrain extends Subsystem {
    private static DriveTrain instance = new DriveTrain();
    public static DriveTrain getInstance() {
        return instance;
    }
    private DriveIO periodic;

    // Drive motors
    public TalonFX forwardRightMotor, rearRightMotor, forwardLeftMotor, rearLeftMotor;
    // Gyroscope
    public PigeonIMU gyro;
    // Solenoid used to change gear
    public DoubleSolenoid transmissionSolenoid;

    public class DriveIO extends PeriodicIO {
        // Read ticks from the left and right drivetrain encoders
        public double leftEncoderTicks, rightEncoderTicks;
        // Joystick X and Y values
        public double xValue, yValue;
        // Motor demands for left and right sides
        public double leftDemand, rightDemand;
        // Error from desired distance for the left and right encoders
        public double leftError, rightError;
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
    }

    public DriveTrain() {
        periodic = new DriveIO();
        transmissionSolenoid = new DoubleSolenoid(
            0,
            PneumaticsModuleType.CTREPCM,
            5, 4
        );
        gyro = new PigeonIMU(1);

        forwardRightMotor = new TalonFX(Constants.DRIVE_FRONT_RIGHT_ID);
        rearRightMotor = new TalonFX(Constants.DRIVE_BACK_RIGHT_ID);
        forwardLeftMotor = new TalonFX(Constants.DRIVE_FRONT_LEFT_ID);
        rearLeftMotor = new TalonFX(Constants.DRIVE_BACK_LEFT_ID);
		forwardRightMotor.setNeutralMode(NeutralMode.Brake);
		rearRightMotor.setNeutralMode(NeutralMode.Brake);
		forwardLeftMotor.setNeutralMode(NeutralMode.Brake);
		rearLeftMotor.setNeutralMode(NeutralMode.Brake);

        forwardLeftMotor.setInverted(true);
        rearLeftMotor.setInverted(true);
    }

    public enum DriveMode {
        OPEN_LOOP,
        TURN,
        MOVE_FORWARD,
        STOPPED
    }

    @Override
    public void readPeriodicInputs() {
        periodic.rawHeading = gyro.getFusedHeading();
        periodic.heading = normalizeHeading(periodic.rawHeading);
        periodic.leftEncoderTicks = forwardLeftMotor.getSelectedSensorPosition();
        periodic.rightEncoderTicks = forwardRightMotor.getSelectedSensorPosition();
        periodic.operatorInput = HIDHelper.getAdjStick(Constants.MASTER_STICK);
        periodic.xValue = periodic.operatorInput[0];
        periodic.yValue = periodic.operatorInput[1];
        // These are important derived values that are also read by actions so they should be updated here
        periodic.headingError = normalizeHeadingError(periodic.targetHeading - periodic.heading);
        periodic.leftError = periodic.targetDistance - periodic.leftEncoderTicks;
        periodic.rightError = periodic.targetDistance - periodic.rightEncoderTicks;
    }

    @Override
    public void writePeriodicOutputs() {
        forwardLeftMotor.set(ControlMode.PercentOutput, periodic.leftDemand);
        rearLeftMotor.set(ControlMode.Follower, Constants.DRIVE_FRONT_LEFT_ID);
        forwardRightMotor.set(ControlMode.PercentOutput, periodic.rightDemand);
        rearRightMotor.set(ControlMode.Follower, Constants.DRIVE_FRONT_RIGHT_ID);
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
    }

    @Override
    public void reset() {
        resetEncoders();
        periodic.rightError = 0;
        periodic.leftError = 0;
        periodic.targetDistance = 0;
        periodic.leftDemand = 0;
        periodic.rightDemand = 0;

        periodic.currentMode = DriveMode.STOPPED;

        transmissionSolenoid.set(Value.kReverse);
        gyro.setFusedHeading(0);
    }

    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {}

            @Override
            public void onLoop(double timestamp) {
                switch (periodic.currentMode) {
                    case OPEN_LOOP:
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
                }
            }

            @Override
            public void onStop(double timestamp) {
                reset();
            }
        });
    }

    public void resetEncoders() {
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

    public double getLeftEncoderDistance() {
        return periodic.leftEncoderTicks;
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(periodic.rawHeading);
    }

    public double getRightEncoderDistance() {
        return periodic.rightEncoderTicks;
    }

    public void setOpenLoop() {
        periodic.currentMode = DriveMode.OPEN_LOOP;
    }

    public void setMoveForward(double distance) {
        periodic.currentMode = DriveMode.MOVE_FORWARD;
        setTargetDistance(distance);
    }

    public void setStopped() {
        periodic.currentMode = DriveMode.STOPPED;
    }

    public void turn() {
        periodic.leftDemand = periodic.headingError * -Constants.ANGLE_KP;
        periodic.rightDemand = periodic.headingError * Constants.ANGLE_KP;

        // Normalize power
        periodic.leftDemand = clampDriveSpeed(periodic.leftDemand,
            Constants.DRIVE_TURN_MINIMUM_SPEED, Constants.DRIVE_TURN_MAXIMUM_SPEED);
        periodic.rightDemand = clampDriveSpeed(periodic.rightDemand,
            Constants.DRIVE_TURN_MINIMUM_SPEED, Constants.DRIVE_TURN_MAXIMUM_SPEED);
    }

    public void moveForward() {
        periodic.driveHeadingCorrect = 0.0;
        periodic.leftDemand = periodic.leftError * Constants.DRIVE_FORWARD_KP;
        periodic.rightDemand = periodic.rightError * Constants.DRIVE_FORWARD_KP;
        
        // Normalize power
        periodic.leftDemand = clampDriveSpeed(periodic.leftDemand, 
            Constants.DRIVE_FORWARD_MINIMUM_SPEED, Constants.DRIVE_FORWARD_MAXIMUM_SPEED);
        periodic.rightDemand = clampDriveSpeed(periodic.rightDemand, 
            Constants.DRIVE_FORWARD_MINIMUM_SPEED, Constants.DRIVE_FORWARD_MAXIMUM_SPEED);

        // Correct for heading error
        periodic.driveHeadingCorrect = periodic.headingError * Constants.DRIVE_FORWARD_HEADING_KP;
        periodic.leftDemand -= periodic.driveHeadingCorrect;
        periodic.rightDemand += periodic.driveHeadingCorrect;

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
    public void openLoop() {
        periodic.leftDemand = periodic.yValue + periodic.xValue;
        periodic.rightDemand = periodic.yValue - periodic.xValue; 

        // Normalize power
        periodic.leftDemand = clampDriveSpeed(periodic.leftDemand, 0.0, 1.0);
        periodic.rightDemand = clampDriveSpeed(periodic.rightDemand, 0.0, 1.0);
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

    public LogData getLogger() {
		return periodic;
	}
}
