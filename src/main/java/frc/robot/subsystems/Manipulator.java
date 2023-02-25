package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import com.playingwithfusion.TimeOfFlight;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class Manipulator extends Subsystem {
	private static Manipulator instance = new Manipulator();
	public static Manipulator getInstance() { return instance; }

	private TalonFX wristMotor;
	private TalonFX intakeMotor;

	public class SuperIO extends Subsystem.PeriodicIO {
		// TOF range from the backstop
		double backstopRange = 0;
		// Motor demand to set intake speed
		double wristMotorPower, intakeMotorPower;
	}

	private SuperIO periodic;

	public Manipulator() {
		periodic = new SuperIO();

		//backstopTOF = new TimeOfFlight(Constants.BACKSTOP_TOF_ID);

		// The left and right side intake wheels. They move the game piece in one direction
		// so we set the left side to be inverted
		wristMotor = new TalonFX(Constants.WRIST_MOTOR_ID);
		intakeMotor = new TalonFX(Constants.INTAKE_MOTOR_ID);
		// TODO: Add intake TOF and HID helper
	}

	public void readPeriodicInputs() {
		//periodic.backstopRange = backstopTOF.getRange();
	}

	public void writePeriodicOutputs() {
		wristMotor.set(ControlMode.PercentOutput, periodic.wristMotorPower);
		intakeMotor.set(ControlMode.PercentOutput, periodic.intakeMotorPower);
	}

	// Set the intake demand to the specified value
	public void setIntakePower(double power) {
		periodic.intakeMotorPower = power;
	}

	public void setWristPower(double power){
		periodic.wristMotorPower = power;
	}
	


	public void reset() {
		//
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
