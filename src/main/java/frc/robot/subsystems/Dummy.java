package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//design pattern for caching periodic writes to avoid hammering the HAL/CAN.
public class Dummy extends Subsystem {
	private static Dummy instance = new Dummy();
	public static Dummy getInstance() { return instance; }
	private DummyIO periodic;

	public enum State {
		kRunning,
		kStopped
	}
	public State state;

	public Dummy() {
		state = State.kStopped;
	}

	/**
	 * Updates all periodic variables and sensors
	 */
	public void readPeriodicInputs() {}

	/**
	 * Writes the periodic outputs to actuators (motors and ect...)
	 */
	public void writePeriodicOutputs() {
		periodic.count++;
	}

	/**
	 * Outputs all logging information to the SmartDashboard
	 */
	public void outputTelemetry() {
		SmartDashboard.putNumber("Dummy count", periodic.count);
	}

	/**
	 * Called to reset and configure the subsystem
	 */
	public void reset() {}

	public class DummyIO extends PeriodicIO {
		public int count = 0;
	}

	public LogData getLogger() {
		return periodic;
	}
}
