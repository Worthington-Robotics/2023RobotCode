package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import java.lang.Math;
import frc.lib.util.HIDHelper;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import java.lang.Math;
import com.ctre.phoenix.sensors.PigeonIMU;
import frc.lib.loops.ILooper;
import frc.lib.loops.Loop;
import frc.robot.Constants;

//design pattern for caching periodic writes to avoid hammering the HAL/CAN.
public class DriveTrain extends Subsystem {
    public TalonFX forwardRightMotor;
    public TalonFX rearRightMotor;
    public TalonFX forwardLeftMotor;
    public TalonFX rearLeftMotor;
    public PigeonIMU gyro;
    public DoubleSolenoid transmissionSolenoid;
    private DriveIO periodic;

    public class DriveIO extends PeriodicIO {
        public double leftEncoderTicks;
        public double rightEncoderTicks;
        public double yValue;
        public double xValue;
        public double rightDemand;
        public double leftDemand;
        public double rawHeading;
        public double heading;
        public double desiredHeading;
        public double[] operatorInput = {0,0,0,0};
        public state currentState = state.OPEN_LOOP;
        public double headingError;
        public double targetDistance;
        public double rightError;
        public double leftError;
        public double powerChange;
    }

    public DriveTrain() {
        periodic = new DriveIO();
        //Move the joystick into the constants files (And all the motor/sensor IDs)
        transmissionSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 0);
        gyro = new PigeonIMU(1);
        forwardRightMotor = new TalonFX(Constants.DRIVE_FRONT_RIGHT_ID);
        rearRightMotor = new TalonFX(Constants.DRIVE_BACK_RIGHT_ID);
        forwardLeftMotor = new TalonFX(Constants.DRIVE_FRONT_LEFT_ID);
        rearLeftMotor = new TalonFX(Constants.DRIVE_BACK_LEFT_ID);

        forwardLeftMotor.setInverted(true);
        rearLeftMotor.setInverted(true);

        //Move this into reset() method
        reset();
      
    }

    public enum state{
        OPEN_LOOP,
        ANGLE_PID,
        MOVE_FORWARD
    }

    private static DriveTrain m_DriveInstance = new DriveTrain();

    public static DriveTrain getInstance() {
        if(m_DriveInstance == null)
            m_DriveInstance = new DriveTrain();
        return m_DriveInstance;
    }

    @Override
    public void readPeriodicInputs() {
        periodic.heading = normalizeHeading(periodic.rawHeading);
        periodic.leftEncoderTicks = forwardLeftMotor.getSelectedSensorPosition();
        periodic.rightEncoderTicks = forwardRightMotor.getSelectedSensorPosition();
        //Update the pidgeon here
        periodic.rawHeading = gyro.getFusedHeading();
        periodic.operatorInput = HIDHelper.getAdjStick(Constants.MASTER_STICK);
        periodic.xValue = periodic.operatorInput[0];
        periodic.yValue = periodic.operatorInput[1];
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
        SmartDashboard.putNumber("Drive/xJoystick", periodic.xValue);
        SmartDashboard.putNumber("Drive/yJoystick", periodic.yValue);
        SmartDashboard.putNumber("Drive/rightEncoder", periodic.rightEncoderTicks);
        SmartDashboard.putNumber("Drive/leftEncoder", periodic.leftEncoderTicks);
        SmartDashboard.putNumber("Drive/Right Demand", periodic.rightDemand);
        SmartDashboard.putNumber("Drive/Left Demand", periodic.leftDemand);
        //Kick out pidgeon heading here too
        SmartDashboard.putNumber("Drive/Heading", periodic.rawHeading);
    }

    @Override
    public void reset() {
        resetEncoders();
        periodic.rightError = 0;
        periodic.leftError = 0;

        transmissionSolenoid.set(Value.kReverse);
        gyro.setFusedHeading(0);
    }

    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {

            @Override
            public void onStart(double timestamp) {
                // TODO Auto-generated method stub

            }

            @Override
            public void onLoop(double timestamp) {
                
                switch (periodic.currentState) {
                    case OPEN_LOOP:
                        setMotorDemands();
                        break;
                    case ANGLE_PID:
                        anglePID();
                    case MOVE_FORWARD:
                        moveForward();
                        
                }           
            }

            @Override
            public void onStop(double timestamp) {
                reset();
            }

        });
    }


    public void resetEncoders(){
        forwardLeftMotor.setSelectedSensorPosition(0);
        forwardRightMotor.setSelectedSensorPosition(0);
        rearLeftMotor.setSelectedSensorPosition(0);
        rearRightMotor.setSelectedSensorPosition(0);
    }

    public void setDesiredHeading(double theta) {
        periodic.desiredHeading = theta;
    }

    public void setTargetDistance(double distance){
        periodic.targetDistance = distance;
    }

    public void setAnglePID (){
        periodic.currentState = state.ANGLE_PID;
    }

    public void setOpenLoop (){
        periodic.currentState = state.OPEN_LOOP;
    }

    public void setMoveForward(){
        periodic.currentState = state.MOVE_FORWARD;
    }

    public void anglePID () {
        periodic.headingError = periodic.desiredHeading - periodic.heading;
        if (Math.abs(periodic.headingError) > 180) {
            if (Math.signum(periodic.headingError) == 1.0) {
                periodic.headingError -= 360.0;
            } else {
                periodic.headingError += 360.0;
            }
        }

        periodic.rightDemand = periodic.headingError * Constants.ANGLE_KP;
        periodic.leftDemand = periodic.headingError * -Constants.ANGLE_KP;
        
        // Normalize
        if (Math.abs(periodic.rightDemand) < 0.09 || Math.abs(periodic.rightDemand) < 0.09) { //Minimum speed
            periodic.rightDemand = Math.signum(periodic.rightDemand) * 0.09;
            periodic.leftDemand = Math.signum(periodic.leftDemand) * 0.09;
        }

        if(Math.abs(periodic.rightDemand) > .5 || Math.abs(periodic.leftDemand) > .5) { //Normalise power
            double norm = Math.max(Math.abs(periodic.rightDemand), Math.abs(periodic.leftDemand));
            periodic.rightDemand = (Math.signum(periodic.rightDemand) * 0.5 ) * Math.abs(periodic.rightDemand / norm);
            periodic.leftDemand = (Math.signum(periodic.leftDemand) * 0.5 )* Math.abs(periodic.leftDemand / norm);
        }
    }

    public void moveForward(){
        periodic.rightError = periodic.targetDistance - periodic.rightEncoderTicks;
        periodic.leftError = periodic.targetDistance - periodic.leftEncoderTicks;
        periodic.powerChange = 0.0;
        periodic.rightDemand = periodic.rightError * Constants.FORWARD_KP;
        periodic.leftDemand = periodic.leftError * Constants.FORWARD_KP;

        periodic.headingError = periodic.desiredHeading - periodic.heading;

        periodic.powerChange = (periodic.headingError) / 45.0; //need to add rollover math
        periodic.rightDemand += periodic.powerChange;
        periodic.leftDemand -= periodic.powerChange;

        if(Math.abs(periodic.rightDemand) > .6 || Math.abs(periodic.leftDemand) > .6) {     //normalizes power
            double norm = Math.max(Math.abs(periodic.rightDemand), Math.abs(periodic.leftDemand));
            periodic.rightDemand = (Math.signum(periodic.rightDemand) * 0.6) * Math.abs(periodic.rightDemand / norm);
            periodic.leftDemand = (Math.signum(periodic.leftDemand) * 0.6 )* Math.abs(periodic.leftDemand / norm);
        }

        if (periodic.rightDemand < 0.09) { //TODO: use signum
            periodic.rightDemand = 0.09;
        } else if (periodic.leftDemand < 0.09){
            periodic.leftDemand = 0.09;
        }
    }

    public double getHeadingError(){
        return periodic.headingError;
    }

    public double getEncoderError(){

        return (periodic.rightError + periodic.leftError) / 2.0;
    }

    public void setMotorDemands(){
        periodic.rightDemand = periodic.yValue - periodic.xValue; 
        periodic.leftDemand = periodic.yValue + periodic.xValue;
    
        if (periodic.rightDemand > 1) {periodic.rightDemand = 1;}                               
        if (periodic.rightDemand < -1) {periodic.rightDemand = -1;}
        if (periodic.leftDemand > 1) {periodic.leftDemand = 1;}
        if (periodic.leftDemand < -1) {periodic.leftDemand = -1;}
    }

    public double normalizeHeading(double heading){
        if((heading % 360) > 180.0){
            return (heading % 360.0)-360.0;
        }
        if((heading % 360) < -180.0){
            return (heading % 360.0)+360.0;
        }
        else return heading % 360.0;
    }

    public LogData getLogger() {
		return periodic;
	}
}