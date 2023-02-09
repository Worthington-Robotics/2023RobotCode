package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.lib.drivers.SimTimeOfFlight;
import frc.lib.util.HIDHelper;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;


//design pattern for caching periodic writes to avoid hammering the HAL/CAN.
public class SuperStructure extends Subsystem {
	private static SuperStructure instance = new SuperStructure();
	public static SuperStructure getInstance() { return instance; }
	private SimTimeOfFlight backstopTOF;
	private TalonFX RightSideWheel;
	private TalonFX LeftSideWheel;
	private TalonFX ConveyerBelt;
	private TalonFX IntakeWheelSpinner;

	double BackstopTOFRange;
	double intakePower;


	public enum State {
		kRunning,
		kStopped
	}
	public State state;

	private int count;

	public SuperStructure() {
		state = State.kStopped;
		backstopTOF = new SimTimeOfFlight(Constants.BACKSTOP_TOF_ID);
		RightSideWheel = new TalonFX(1); 
		

		//add intake TOF
		//Add HID helper
	}

	/**
	 * Updates all periodic variables and sensors
	 */
	public void readPeriodicInputs() {
		double BackstopTOFRange = backstopTOF.getRange();
	}

	/**
	 * Writes the periodic outputs to actuators (motors and ect...)
	 */
	public void writePeriodicOutputs() {
		if (BackstopTOFRange < 10){
			RightSideWheel.set(ControlMode.PercentOutput, intakePower);
			LeftSideWheel.set(ControlMode.PercentOutput, -intakePower);
			ConveyerBelt.set(ControlMode.PercentOutput, intakePower);
			IntakeWheelSpinner.set(ControlMode.PercentOutput, intakePower);
		}
	}

	/**
	 *
	 *
	 */
	public void setIntakePower(double newPower) {
		intakePower = newPower
	}


	/**
	 * Outputs all logging information to the SmartDashboard
	 */
	public void outputTelemetry() {
		SmartDashboard.putNumber("SuperStructure/Intake_Power", Intake_Power);
	}

	/**
	 * Called to reset and configure the subsystem
	 */
	public void reset() {}
}
