package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.lib.util.HIDHelper;
import frc.lib.loops.ILooper;
import frc.lib.loops.Loop;

public class Manipulator extends Subsystem {
	private static Manipulator instance = new Manipulator();
	public static Manipulator getInstance() { return instance; }
	public ManipulatorIO periodic;

	private TalonFX wristMotor;
	private TalonFX intakeMotor;
	public State state;

	public enum State {
		OPEN_LOOP,
		OPEN_CLOSED_LOOP,
		CLOSED_LOOP
    }

	public class ManipulatorIO extends PeriodicIO {
		// Motor demand to set intake speed
		double wristMotorPower;
		double intakeMotorPower;
		double rawWristMotorPower;
		double desiredWristEncoder;
		double desiredArmLength;
	}

	public Manipulator() {
		periodic = new ManipulatorIO();

		//backstopTOF = new TimeOfFlight(Constants.BACKSTOP_TOF_ID);

		// The left and right side intake wheels. They move the game piece in one direction
		// so we set the left side to be inverted
		wristMotor = new TalonFX(Constants.WRIST_MOTOR_ID, "Default Name");
		wristMotor.setNeutralMode(NeutralMode.Brake);
		intakeMotor = new TalonFX(Constants.INTAKE_MOTOR_ID, "Default Name");
		intakeMotor.setNeutralMode(NeutralMode.Brake);
		// TODO: Add intake TOF
	}

	public void readPeriodicInputs() {
		periodic.rawWristMotorPower = HIDHelper.getAxisMapped(Constants.MASTER.getRawAxis(3), 1,0);
	}

	public void writePeriodicOutputs() {
		wristMotor.set(ControlMode.PercentOutput, periodic.wristMotorPower);
		intakeMotor.set(ControlMode.PercentOutput, periodic.intakeMotorPower);
		wristMotor.set(ControlMode.PercentOutput, periodic.desiredWristEncoder);
	}

	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(new Loop(){

			@Override
			public void onStart(double timestamp) {
				reset();
			}

			@Override
			public void onLoop(double timestamp) {
				switch (state) {
					case OPEN_LOOP:
						setWristPower(periodic.rawWristMotorPower);
						break;
					case OPEN_CLOSED_LOOP:
						periodic.desiredArmLength = convertRawWristPowerIntoEncoder(periodic.rawWristMotorPower);
						wristAnglePID();
					case CLOSED_LOOP:
						wristAnglePID();
				}
			}

			@Override
			public void onStop(double timestamp) {
				reset();
				
			}
			
		});
		
	}

	//convert joystick values into motor powers
	public double convertRawWristPowerIntoEncoder(double inputPower){
		return inputPower * 1000;
	}

	// Set the intake demand to the specified value
	public void setIntakePower(double power) {
		periodic.intakeMotorPower = power;
	}

	public void setWristPower(double power){
		periodic.wristMotorPower = power;
	}

	//PID
	
	public void wristAnglePID(){
		wristMotor.config_kP(0, Constants.WRIST_PIVOT_KP);
	}


	public void reset() {
	}

	// ### Telemetry ###

	public void outputTelemetry() {
		SmartDashboard.putNumber("Manipulator/IntakePower", periodic.intakeMotorPower);
		SmartDashboard.putNumber("Manipulator/WristPower", periodic.wristMotorPower);
	}

	public LogData getLogger() {
		return periodic;
	}
}
