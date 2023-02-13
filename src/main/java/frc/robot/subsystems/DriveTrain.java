package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.lib.util.HIDHelper;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
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

    class DriveIO extends PeriodicIO {
        public double leftEncoderTicks;
        public double rightEncoderTicks;
        public double yValue;
        public double xValue;
        public double rightDemand;
        public double leftDemand;
        public double heading;
        public double[] operatorInput = {0,0,0,0};
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
    }

    private static DriveTrain m_DriveInstance = new DriveTrain();

    public static DriveTrain getInstance() {
        if(m_DriveInstance == null)
            m_DriveInstance = new DriveTrain();
        return m_DriveInstance;
    }

    @Override
    public void readPeriodicInputs() {
        periodic.leftEncoderTicks = forwardLeftMotor.getSelectedSensorPosition();
        periodic.rightEncoderTicks = forwardRightMotor.getSelectedSensorPosition();
        //Update the pidgeon here
        periodic.heading = gyro.getFusedHeading();
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
        SmartDashboard.putNumber("Drive/Heading", periodic.heading);
    }

    @Override
    public void reset() {
        resetEncoders();

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
        rearLeftMotor.setSelectedSensorPosition(0);
        rearRightMotor.setSelectedSensorPosition(0);
    }

    public void setMotorDemands(){
        periodic.rightDemand = periodic.yValue - periodic.xValue; 
        periodic.leftDemand = periodic.yValue + periodic.xValue;
    
        if (periodic.rightDemand > 1) {periodic.rightDemand = 1;}                               
        if (periodic.rightDemand < -1) {periodic.rightDemand = -1;}
        if (periodic.leftDemand > 1) {periodic.leftDemand = 1;}
        if (periodic.leftDemand < -1) {periodic.leftDemand = -1;}
    }
}
