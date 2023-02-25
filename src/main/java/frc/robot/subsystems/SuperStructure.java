package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import com.playingwithfusion.TimeOfFlight;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class SuperStructure extends Subsystem {
	private static SuperStructure instance = new SuperStructure();
	public static SuperStructure getInstance() { return instance; }

	//private TimeOfFlight backstopTOF;
	private TalonFX wristMotor;
	private TalonFX intakeMotor;

	public class SuperIO extends Subsystem.PeriodicIO {
		// TOF range from the backstop
		double backstopRange = 0;
		// Motor demand to set intake speed
		double wristMotorPower, intakeMotorPower;
	}

	private SuperIO periodic;

	public SuperStructure() {
		periodic = new SuperIO();

		//backstopTOF = new TimeOfFlight(Constants.BACKSTOP_TOF_ID);

		// The left and right side intake wheels. They move the game piece in one direction
		// so we set the left side to be inverted
		wristMotor = new TalonFX(Constants.WRIST_MOTOR_ID, "Default Name");
		wristMotor.setNeutralMode(NeutralMode.Brake);
		intakeMotor = new TalonFX(Constants.INTAKE_MOTOR_ID, "Default Name");
		intakeMotor.setNeutralMode(NeutralMode.Brake);
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
		SmartDashboard.putNumber("SuperStructure/IntakePower", periodic.intakeMotorPower);
		SmartDashboard.putNumber("SuperStructure/WristPower", periodic.wristMotorPower);
	}

	public LogData getLogger() {
		return periodic;
	}
}
