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
    public Joystick joystick;
    public DoubleSolenoid transmissionSolenoid;

    public double leftEncoderTicks;
    public double rightEncoderTicks;
    public double yValue;
    public double xValue;
    public double rightDemand;
    public double leftDemand;
    public double heading;


    public DriveTrain() {
        //Move the joystick into the constants files (And all the motor/sensor IDs)
        joystick = Constants.SECOND;
        transmissionSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 0);
        gyro = new PigeonIMU(1);
        forwardRightMotor = new TalonFX(Constants.DRIVE_FRONT_RIGHT_ID);
        rearRightMotor = new TalonFX(Constants.DRIVE_BACK_RIGHT_ID);
        forwardLeftMotor = new TalonFX(Constants.DRIVE_FRONT_LEFT_ID);
        rearLeftMotor = new TalonFX(Constants.DRIVE_BACK_LEFT_ID);

        //Move this into reset() method
        reset();
      
    }

    private static DriveTrain m_DriveInstance = new DriveTrain();

    public static DriveTrain getInstance() {
        return m_DriveInstance;
    }

    @Override
    public void readPeriodicInputs() {
        leftEncoderTicks = forwardLeftMotor.getSelectedSensorPosition();
        rightEncoderTicks = forwardRightMotor.getSelectedSensorPosition();
        //Update the pidgeon here
        heading = gyro.getFusedHeading();
        yValue = joystick.getRawAxis(1);
        xValue = joystick.getRawAxis(0);
        
    }

    @Override
    public void writePeriodicOutputs() {
        forwardLeftMotor.set(ControlMode.PercentOutput, leftDemand);
        rearLeftMotor.set(ControlMode.Follower, 4);
        forwardRightMotor.set(ControlMode.PercentOutput, rightDemand);
        rearRightMotor.set(ControlMode.Follower, 1);
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Right Demand", rightDemand);
        SmartDashboard.putNumber("Left Demand", leftDemand);
        //Kick out pidgeon heading here too
        SmartDashboard.putNumber("Heading", heading);
    }

    @Override
    public void reset() {
        forwardLeftMotor.setInverted(true);
        rearLeftMotor.setInverted(true);

        forwardLeftMotor.setSelectedSensorPosition(0);
        forwardRightMotor.setSelectedSensorPosition(0);
        rearLeftMotor.setSelectedSensorPosition(0);
        rearRightMotor.setSelectedSensorPosition(0);

        transmissionSolenoid.set(Value.kReverse);
        gyro.setFusedHeading(0);

        resetEncoders();
    }

    
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {

            @Override
            public void onStart(double timestamp) {
                // TODO Auto-generated method stub

            }

            @Override
            public void onLoop(double timestamp) {
                  setMotorDemands();             
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
    }

    public void setMotorDemands(){
        rightDemand = -yValue - xValue; 
        leftDemand = -yValue + xValue;
       
        if(rightDemand < Constants.deadZone && rightDemand > - Constants.deadZone) {
            rightDemand = 0;
        }  
        if(leftDemand < Constants.deadZone && leftDemand > - Constants.deadZone) {
            leftDemand = 0;
        }
    
        if (rightDemand > 1) {rightDemand = 1;}                               
        if (rightDemand < -1) {rightDemand = -1;}
        if (leftDemand > 1) {leftDemand = 1;}
        if (leftDemand < -1) {leftDemand = -1;}
    }

   


}
