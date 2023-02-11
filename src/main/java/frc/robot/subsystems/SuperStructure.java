package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.playingwithfusion.TimeOfFlight;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class SuperStructure extends Subsystem {
	private static SuperStructure instance = new SuperStructure();
	public static SuperStructure getInstance() { return instance; }

	private TimeOfFlight backstopTOF;
	private TalonFX leftSideWheel;
	private TalonFX rightSideWheel;
	private TalonFX conveyorBelt;
	private TalonFX intakeWheelSpinner;

	public class SuperIO extends Subsystem.PeriodicIO {
		// TOF range from the backstop
		double backstopRange = 0.0d;
		// Motor demand to set intake speed
		double power = 0.0d;
	}
	private SuperIO periodic;

	public SuperStructure() {
		periodic = new SuperIO();

		backstopTOF = new TimeOfFlight(Constants.BACKSTOP_TOF_ID);

		// The left and right side intake wheels. They move the game piece in one direction
		// so we set the left side to be inverted
		leftSideWheel = new TalonFX(Constants.INTAKE_LEFT_WHEEL_ID);
		leftSideWheel.setInverted(true);
		rightSideWheel = new TalonFX(Constants.INTAKE_RIGHT_WHEEL_ID);

		conveyorBelt = new TalonFX(Constants.INTAKE_CONVEYOR_ID);
		intakeWheelSpinner = new TalonFX(Constants.INTAKE_SPINNER_ID);

		// TODO: Add intake TOF and HID helper
	}

	public void readPeriodicInputs() {
		periodic.backstopRange = backstopTOF.getRange();
	}

	public void writePeriodicOutputs() {
		leftSideWheel.set(ControlMode.PercentOutput, periodic.power * Constants.SIDE_WHEELS_MULTIPLIER );
		rightSideWheel.set(ControlMode.PercentOutput, periodic.power * Constants.SIDE_WHEELS_MULTIPLIER);
		conveyorBelt.set(ControlMode.PercentOutput, periodic.power * Constants.CONVEYER_BELT_MULTIPLIER);
		intakeWheelSpinner.set(ControlMode.PercentOutput, periodic.power * Constants.INTAKE_WHEEL_SPINNER_MULTIPLIER);
	}

	// Set the intake demand to the specified value
	public void setIntakePower(double power) {
		periodic.power = power;
	}

	// Returns if the game piece has completed its intake cycle
	public boolean isFinished() {
		return (periodic.backstopRange >= Constants.INTAKE_BACKSTOP_DISTANCE);
	}

	public void reset() {
		leftSideWheel.setInverted(true);
		periodic = new SuperIO();
	}

	// ### Telemetry ###

	public void outputTelemetry() {
		SmartDashboard.putNumber("SuperStructure/IntakePower", periodic.power);
	}

	public LogData getLogger() {
		return periodic;
	}
}
