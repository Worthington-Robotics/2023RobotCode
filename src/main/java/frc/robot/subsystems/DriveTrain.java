package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import java.lang.Math;
import com.ctre.phoenix.sensors.PigeonIMU;
//                                                                       Motor Layout
public class DriveTrain extends Subsystem {
    public TalonFX forwardRightMotor;
    private TalonFX rearRightMotor;
    public TalonFX forwardLeftMotor;
    private TalonFX rearLeftMotor;
    public PigeonIMU gyro;
    public Joystick joy;
    private DoubleSolenoid transmissionSolenoid;
    private int state;
    public double startTime;

    // This is for the arcturn, all the delta variables
    private double deltaTime;
    private double deltaEncoderLeft;
    private double deltaEncoderRight;
    public double sDeltaEncoderLeft;
    public double sDeltaEncoderRight;
    public double sDeltaHeading;
    private double deltaHeading;

    public double timeDelay = 1.5;
    private double ticksPerInch = 1705.0; // TODO: More precise value
    private double[] coordinates = {0.0, 0.0}; // Start point will be 0, 0
    
    private double deadZone = 0.05; // Adjustable deadzone

    public DriveTrain(Joystick joystick) {
        this.joy = joystick;
        transmissionSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 0);
        gyro = new PigeonIMU(1);
        forwardRightMotor = new TalonFX(1); //  |1        3|
        rearRightMotor = new TalonFX(2); //     |          |
        forwardLeftMotor = new TalonFX(4); //   |          |
        rearLeftMotor = new TalonFX(3); //      |2        4|

        forwardLeftMotor.setInverted(true);
        rearLeftMotor.setInverted(true);
        forwardLeftMotor.setSelectedSensorPosition(0);
        forwardRightMotor.setSelectedSensorPosition(0);
        transmissionSolenoid.set(Value.kReverse);
        gyro.setFusedHeading(0);

        state = 0;
    }
    
    @Override
    public void teleop() {
        // 1 is y, 0 is axis, negative because y is inverted on flight stick
        double rightSpeed = -joy.getRawAxis(1)-joy.getRawAxis(0);
        double leftSpeed = -joy.getRawAxis(1)+joy.getRawAxis(0);  // speed = y + x

        Boolean transmissionSwitch = joy.getRawButton(2);
        SmartDashboard.putBoolean("TransmissionSwitch", transmissionSwitch);
    
        if(rightSpeed < deadZone && rightSpeed > -deadZone) {rightSpeed = 0;}             // Deadzone commands
        if(leftSpeed < deadZone && leftSpeed > -deadZone) {leftSpeed = 0;}
    
        if (rightSpeed > 1) {rightSpeed = 1;}                                             // Normalize Speed
        if (rightSpeed < -1) {rightSpeed = -1;}
        if (leftSpeed > 1) {leftSpeed = 1;}
        if (leftSpeed < -1) {leftSpeed = -1;}

        if (transmissionSwitch){
            if (transmissionSolenoid.get() == Value.kReverse || transmissionSolenoid.get() == Value.kOff){
                transmissionSolenoid.set(Value.kForward);
            }
        } else {
            if (transmissionSolenoid.get() == Value.kForward || transmissionSolenoid.get() == Value.kOff) {
                transmissionSolenoid.set(Value.kReverse);
            }
        }

        forwardRightMotor.set(ControlMode.PercentOutput, rightSpeed);
        rearRightMotor.set(ControlMode.PercentOutput, rightSpeed);
        SmartDashboard.putNumber("Right_speed", rightSpeed);
    
        forwardLeftMotor.set(ControlMode.PercentOutput, leftSpeed);
        rearLeftMotor.set(ControlMode.PercentOutput, leftSpeed);
        SmartDashboard.putNumber("Left_speed", leftSpeed);

        double heading = gyro.getFusedHeading() % 360;
        SmartDashboard.putNumber("heading", heading);
    }
    
    public void autonomous() {
        getPose();

        double heading = gyro.getFusedHeading() % 360; // Heading is any value between 0 and 360
        double leftEncoder = forwardLeftMotor.getSelectedSensorPosition();
        double rightEncoder = forwardRightMotor.getSelectedSensorPosition();
        double rightVelocity = forwardRightMotor.getSelectedSensorVelocity();
        double leftVelocity = forwardLeftMotor.getSelectedSensorVelocity();
        

        SmartDashboard.putNumber("heading", heading);
        SmartDashboard.putNumber("leftEncoder", leftEncoder);
        SmartDashboard.putNumber("rightEncoder", rightEncoder);
        SmartDashboard.putNumber("leftVelocity", leftVelocity);
        SmartDashboard.putNumber("rightVelocity", rightVelocity);

         switch (state) {
            // Place object
            case 0: 
                if ((Timer.getFPGATimestamp() - startTime) > 2.5) {
                    state++;
                }
                break;
            case 1: // turn Right
                if (goForwards(80000.0, 0)) {
                    state++;
                    startTime = Timer.getFPGATimestamp();
                }
                break;
            case 2:
                if (turn(-90.0)){
                    state++;
                }
                break;
            case 3:
                if (goForwards(368000.0, -90.0)) {
                    state++;
                }
                startTime = Timer.getFPGATimestamp();
                break;
            case 4: 
                if (turn(90.0)) {
                    state++;
                }
                break;
            case 5:
                if (goForwards(368000.0, 90.0)) {
                    state++;
                }
                startTime = Timer.getFPGATimestamp();
                break;
            case 6:
                break;
        }
        SmartDashboard.putNumber("state", state);
    }

    public boolean goForwards(double targetDistance, double targetHeading) {
        double leftEncoder = forwardLeftMotor.getSelectedSensorPosition();
        double rightEncoder = forwardRightMotor.getSelectedSensorPosition();
        double rightErr = targetDistance - rightEncoder;
        double leftErr = targetDistance - leftEncoder;
        double heading = gyro.getFusedHeading() % 360; //heading is any value between 0 and 360
        double powerChange = 0.0; 
        double kp = 1.0/100000.0;
        double rightPower = rightErr * kp;
        double leftPower = leftErr * kp;

        powerChange = (targetHeading - heading) / 45.0; //need to add rollover math
        rightPower += powerChange;
        leftPower -= powerChange;

        if(Math.abs(rightPower) > .8 || Math.abs(leftPower) > .8) {
            double norm = Math.max(Math.abs(rightPower), Math.abs(leftPower));
            rightPower = (Math.signum(rightPower) * 0.8) * Math.abs(rightPower / norm);
            leftPower = (Math.signum(leftPower) * 0.8)* Math.abs(leftPower / norm);
        }

        if ((Math.abs(rightPower) < 0.09) || (Math.abs(leftPower) < 0.09)) {
            rightPower = Math.signum(rightPower) * 0.09;
            leftPower = Math.signum(leftPower) * 0.09;
        } //1705 per inch ~ roughly

        SmartDashboard.putNumber("RightPower", rightPower);
        SmartDashboard.putNumber("LeftPower", leftPower);
        SmartDashboard.putNumber("rightErr", rightErr);
        SmartDashboard.putNumber("leftErr", leftErr);
        SmartDashboard.putNumber("powerchange", powerChange);
        SmartDashboard.putNumber("FPGA", startTime);

        forwardRightMotor.set(ControlMode.PercentOutput, rightPower);
        rearRightMotor.set(ControlMode.PercentOutput, rightPower);
        forwardLeftMotor.set(ControlMode.PercentOutput, leftPower);
        rearLeftMotor.set(ControlMode.PercentOutput, leftPower);

        if (Math.abs(targetDistance - leftEncoder) < 3000.0) {
            stopMotors();
            resetEncoders();
            return true;
        } else {
            return false;
        }
    }

    // Updates the global variable coordinates[] in ticks
    public void getPose() {
        double heading = gyro.getFusedHeading();
        
        double deltaDistance = (deltaEncoderRight + deltaEncoderLeft) / 2;
        double deltaX = (deltaDistance) * Math.cos(heading);
        double deltaY = (deltaDistance) * Math.sin(heading);

        coordinates[0] += deltaX;
        coordinates[1] += deltaY;
    }

    public double normalizeHeading(double heading) {
        if ((heading % 360) > 180.0) {
            return (heading % 360.0) - 360.0;
        }
        if ((heading % 360) < -180.0) {
            return (heading % 360.0) + 360.0;
        }
        return heading % 360.0;
    }

    public boolean arcTurn(double desiredX, double desiredY){
        double kp = 1 / 300.0;
        double currentX = coordinates[0]; // What x is at that moment
        double currentY = coordinates[1]; // What y is at that moment
        double leftPower; // Set the left motor to a certain power for the arc turn
        double rightPower; // Set the right motor to certain power
        double robotDesiredHeading;
        double robotCurrentHeading;
        double headingError;


        double xError = desiredX - currentX;
        double yError = desiredY - currentY;
        // How far off the robot is from the target
        double distanceError = Math.sqrt(Math.pow(xError, 2) + Math.pow(yError, 2)); 

        // Based on the relative location of the robot to the target point, get the desired heading
        // If the robot is not aligned with the target horizontally
        if (xError != 0) {
            robotDesiredHeading = Math.atan(yError / xError) * (180 / Math.PI);
            robotCurrentHeading = gyro.getFusedHeading() % 360;
            headingError = robotDesiredHeading - robotCurrentHeading;
        // If the target is horizontally right in front of the robot
        } else {
            if (yError < 0) {
                robotDesiredHeading = -90.0;
                robotCurrentHeading = gyro.getFusedHeading() % 360;
                headingError = robotDesiredHeading - robotCurrentHeading;
            } else {
                robotDesiredHeading = 90.0;
                robotCurrentHeading = gyro.getFusedHeading() % 360;
                headingError = robotDesiredHeading - robotCurrentHeading; 
            }
        }

        // Based on how far the robot is from desired heading, calculate heading power
        // The farther the robot heading is from desired heading, the larger the difference between the right and left motors
        double headingPower = Math.abs(headingError * kp);
        double distancePower = distanceError * kp;
        
        // If the target point is in front of the robot
        if (xError >= 0) {
            if(headingError < 0) { //if the target point is right of the robot
                leftPower = distancePower + headingPower;
                rightPower =  distancePower;
            } else if (headingError > 0) { //if the target is left of the robot
                leftPower = distancePower;
                rightPower =  distancePower + headingPower;
            } else { //if the target point is in front of the robot
                leftPower = distancePower;
                rightPower =  distancePower;
            }
        } else { // if the target point is behind the robot
            if(headingError > 0){ // if the target is right of the robot
                leftPower = distancePower + headingPower;
                rightPower =  distancePower;
                leftPower *= -1;
                rightPower *= -1;
            } else if (headingError < 0){ // if the target is left of the robot
                leftPower = distancePower;
                rightPower =  distancePower + headingPower;
                leftPower *= -1;
                rightPower *= -1;
            } else { // If the target is right behind the robot
                leftPower = distancePower;
                rightPower =  distancePower;
                leftPower *= -1;
                rightPower *= -1;
            }
        }

        // Lower cap
        if (Math.abs(leftPower) <= 0.09) {
            leftPower = Math.signum(leftPower) * 0.09;
        }
        if (Math.abs(rightPower) <= 0.09) {
            rightPower = Math.signum(rightPower) * 0.09;
        }
        
        if (Math.abs(rightPower) > .75 || Math.abs(leftPower) > .75) { // Max speed
            double norm = Math.max(Math.abs(rightPower), Math.abs(leftPower)); // TODO: Phillip explain
            rightPower = (Math.signum(rightPower) * 0.75) * Math.abs(rightPower / norm);
            leftPower = (Math.signum(leftPower) * 0.75 )* Math.abs(leftPower / norm);
        }

        
        forwardLeftMotor.set(ControlMode.PercentOutput, leftPower); 
        rearLeftMotor.set(ControlMode.PercentOutput, leftPower); 
        forwardRightMotor.set(ControlMode.PercentOutput, rightPower);
        rearRightMotor.set(ControlMode.PercentOutput, rightPower);

        SmartDashboard.putNumber("desired heading", robotDesiredHeading);
        SmartDashboard.putNumber("heading error", headingError);
        SmartDashboard.putNumber("x goal", desiredX);
        SmartDashboard.putNumber("y goal", desiredY);
        SmartDashboard.putNumber("x error", xError);
        SmartDashboard.putNumber("y error", yError);
        SmartDashboard.putNumber("distance error", distanceError);
        SmartDashboard.putNumber("leftPower", leftPower);
        SmartDashboard.putNumber("rightPower", rightPower);
        
        if (distanceError <= 10) {
            return true;
        } else {
            return false;
        }
    }

    public boolean turn(double targetHeading){
        double heading = normalizeHeading(gyro.getFusedHeading()); // Heading is any value between 0 and 360
        double kp = 1.0/60.0;
        double error = targetHeading - heading;
        if (Math.abs(error) > 180) {
            if (Math.signum(error) == 1.0) {
                error -= 360.0;
            } else {
                error += 360.0;
            }
        }

        double rightPower = error * kp;
        double leftPower = error * -kp;

        if (Math.abs(rightPower) < 0.09 || Math.abs(leftPower) < 0.09) {
            rightPower = Math.signum(rightPower) * 0.09;
            leftPower = Math.signum(leftPower) * 0.09;
        } //1705 per inch ~ roughly

        if (Math.abs(rightPower) > .5 || Math.abs(leftPower) > .5) {
            double norm = Math.max(Math.abs(rightPower), Math.abs(leftPower));
            rightPower = (Math.signum(rightPower) * 0.5 ) * Math.abs(rightPower / norm);
            leftPower = (Math.signum(leftPower) * 0.5 )* Math.abs(leftPower / norm);
        }

        SmartDashboard.putNumber("LeftPower", leftPower);
        SmartDashboard.putNumber("RightPower", rightPower);

        forwardRightMotor.set(ControlMode.PercentOutput, rightPower);
        rearRightMotor.set(ControlMode.PercentOutput, rightPower);
        forwardLeftMotor.set(ControlMode.PercentOutput, leftPower);
        rearLeftMotor.set(ControlMode.PercentOutput, leftPower);
        
        heading = normalizeHeading(gyro.getFusedHeading());
        if (Math.abs(targetHeading - heading) < 2.0 && (Timer.getFPGATimestamp() - startTime) > timeDelay) {
            stopMotors();
            resetEncoders();
            return true;
        } else {
            SmartDashboard.putNumber("ResultHeading", heading);
            return false;
        }
    }
    
    public void stopMotors() {
        forwardRightMotor.set(ControlMode.PercentOutput, 0);
        rearRightMotor.set(ControlMode.PercentOutput, 0);
        forwardLeftMotor.set(ControlMode.PercentOutput, 0);
        rearLeftMotor.set(ControlMode.PercentOutput, 0);
    }

    public void resetEncoders() {
        forwardRightMotor.setSelectedSensorPosition(0);
        forwardLeftMotor.setSelectedSensorPosition(0);
    }
}
