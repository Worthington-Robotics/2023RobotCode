package frc.robot.subsystems;
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
        public double targetHeading;
        // Error from desired heading
        public double headingError;
        // Desired distance to reach
        public double targetDistance;
        // Axis values from the joystick
        public double[] operatorInput = { 0, 0, 0, 0 };
        // Current drive mode
        public DriveMode currentMode = DriveMode.OPEN_LOOP;
        public double powerChange;
    }

    public DriveTrain() {
        periodic = new DriveIO();
        // TODO: Move the joystick into the constants files (And all the motor/sensor IDs)
        transmissionSolenoid = new DoubleSolenoid(
            PneumaticsModuleType.CTREPCM,
            1, 0
        );
        gyro = new PigeonIMU(1);

        forwardRightMotor = new TalonFX(Constants.DRIVE_FRONT_RIGHT_ID);
        rearRightMotor = new TalonFX(Constants.DRIVE_BACK_RIGHT_ID);
        forwardLeftMotor = new TalonFX(Constants.DRIVE_FRONT_LEFT_ID);
        rearLeftMotor = new TalonFX(Constants.DRIVE_BACK_LEFT_ID);

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
        periodic.headingError = periodic.targetHeading - periodic.heading;
        if (Math.abs(periodic.headingError) > 180) {
            if (Math.signum(periodic.headingError) == 1.0) {
                periodic.headingError -= 360.0;
            } else {
                periodic.headingError += 360.0;
            }
        }
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
    }

    public void setTargetDistance(double distance) {
        periodic.targetDistance = distance;
    }

    // Set drive encoder error prior to moving towards a setpoint
    public void setEncoderError(double error) {
        periodic.leftError = error;
        periodic.rightError = error;
    }

    public void setHeadingError(double error) {
        periodic.headingError = error;
    }

    public void setTurning() {
        periodic.currentMode = DriveMode.TURN;
    }

    public void setOpenLoop() {
        periodic.currentMode = DriveMode.OPEN_LOOP;
    }

    public void setMoveForward() {
        periodic.currentMode = DriveMode.MOVE_FORWARD;
    }

    public void setStopped() {
        periodic.currentMode = DriveMode.STOPPED;
    }

    public void turn() {
        periodic.leftDemand = periodic.headingError * -Constants.ANGLE_KP;
        periodic.rightDemand = periodic.headingError * Constants.ANGLE_KP;
        
        // Minimum speed
        if (Math.abs(periodic.leftDemand) < Constants.DRIVE_TURN_MINIMUM_SPEED ||
            Math.abs(periodic.rightDemand) < Constants.DRIVE_TURN_MINIMUM_SPEED)
        {
            periodic.leftDemand = Math.signum(periodic.leftDemand) * 0.09;
            periodic.rightDemand = Math.signum(periodic.rightDemand) * 0.09;
        }

        // Normalize power
        if (Math.abs(periodic.leftDemand) > .5 || Math.abs(periodic.rightDemand) > .5) {
            final double norm = Math.max(Math.abs(periodic.leftDemand), Math.abs(periodic.rightDemand));
            periodic.leftDemand = (Math.signum(periodic.leftDemand) * 0.5 ) * Math.abs(periodic.leftDemand / norm);
            periodic.rightDemand = (Math.signum(periodic.rightDemand) * 0.5 ) * Math.abs(periodic.rightDemand / norm);
        }
    }

    public void moveForward() {
        periodic.powerChange = 0.0;
        periodic.leftDemand = periodic.leftError * Constants.FORWARD_KP;
        periodic.rightDemand = periodic.rightError * Constants.FORWARD_KP;

        //TODO: Add rollover math
        periodic.powerChange = (periodic.headingError) / 45.0;
        periodic.leftDemand -= periodic.powerChange;
        periodic.rightDemand += periodic.powerChange;

        // Normalize power
        if (Math.abs(periodic.rightDemand) > .6 || Math.abs(periodic.leftDemand) > .6) {
            double norm = Math.max(Math.abs(periodic.rightDemand), Math.abs(periodic.leftDemand));
            periodic.rightDemand = (Math.signum(periodic.rightDemand) * 0.6) * Math.abs(periodic.rightDemand / norm);
            periodic.leftDemand = (Math.signum(periodic.leftDemand) * 0.6 )* Math.abs(periodic.leftDemand / norm);
        }

        if (periodic.rightDemand < 0.9 || periodic.rightDemand > -0.9) {
            periodic.rightDemand = Math.signum(periodic.rightDemand) * 0.9;
        } else if (periodic.leftDemand < 0.9 || periodic.leftDemand > -0.9) {
            periodic.leftDemand = 0.09;
        }
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
        periodic.rightDemand = periodic.yValue - periodic.xValue; 
        periodic.leftDemand = periodic.yValue + periodic.xValue;
    
        if (periodic.rightDemand > 1) { periodic.rightDemand = 1;}
        if (periodic.rightDemand < -1) { periodic.rightDemand = -1; }
        if (periodic.leftDemand > 1) { periodic.leftDemand = 1; }
        if (periodic.leftDemand < -1) { periodic.leftDemand = -1; }
    }

    public double normalizeHeading(double heading) {
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

    public LogData getLogger() {
		return periodic;
	}
}
