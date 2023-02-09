package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//design pattern for caching periodic writes to avoid hammering the HAL/CAN.
public class Dummy extends Subsystem {
	private static Dummy instance = new Dummy();
	public static Dummy getInstance() { return instance; }

	public enum State {
		kRunning,
		kStopped
	}
	private State state;

	public void setState(State _state) {
		state = _state;
	}

	public int getCount() {
		return count;
	}

	private int count;

	public Dummy() {
		state = State.kRunning;
	}

	/**
	 * Updates all periodic variables and sensors
	 */
	public void readPeriodicInputs() {}

	/**
	 * Writes the periodic outputs to actuators (motors and ect...)
	 */
	public void writePeriodicOutputs() {
		if (state == State.kRunning) {
			count++;
		}
	}

	/**
	 * Outputs all logging information to the SmartDashboard
	 */
	public void outputTelemetry() {
		SmartDashboard.putNumber("Dummy count", count);
	}

	/**
	 * Called to reset and configure the subsystem
	 */
	public void reset() {}
}
