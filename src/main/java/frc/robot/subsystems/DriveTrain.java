package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.HIDHelper;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import java.lang.Math;
import com.ctre.phoenix.sensors.PigeonIMU;

public class DriveTrain extends Subsystem{
    public TalonFX forwardRightMotor;
    public TalonFX rearRightMotor;
    public TalonFX forwardLeftMotor;
    public TalonFX rearLeftMotor;
    public PigeonIMU gyro;
    public Joystick joystick;
    public DoubleSolenoid transmissionSolenoid;
    private DriveIO periodic;

    //Constants
    private double leftPosTicks;
    private double rightPosTicks;


      // construct one and only 1 instance of this class
    private static DriveTrain m_DriveInstance = new DriveTrain();

    public static DriveTrain getInstance() {
        return m_DriveInstance;
    }

    public DriveTrain() {
        joystick = new Joystick(1);
        transmissionSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 0);
        gyro = new PigeonIMU(1);
        forwardRightMotor = new TalonFX(1);
        rearRightMotor = new TalonFX(2);
        forwardLeftMotor = new TalonFX(4);
        rearLeftMotor = new TalonFX(3);

        forwardLeftMotor.setInverted(true);
        rearLeftMotor.setInverted(true);

        transmissionSolenoid.set(Value.kReverse);
        gyro.setFusedHeading(0);

        leftPosTicks = 0;
        rightPosTicks = 0;

        
    }


    public class DriveIO extends PeriodicIO {

    }

    @Override
    public void readPeriodicInputs() {
        leftPosTicks = forwardLeftMotor.getSelectedSensorPosition();
        rightPosTicks = forwardRightMotor.getSelectedSensorPosition();
    }

    @Override
    public void writePeriodicOutputs() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void outputTelemetry() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void reset() {
        resetEncoders();
    }

    private void resetEncoders() {
        forwardLeftMotor.setSelectedSensorPosition(0);
        rearLeftMotor.setSelectedSensorPosition(0);
        forwardRightMotor.setSelectedSensorPosition(0);
        rearRightMotor.setSelectedSensorPosition(0);
    }
    
}
