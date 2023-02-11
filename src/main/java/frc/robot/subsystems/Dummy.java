package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

//design pattern for caching periodic writes to avoid hammering the HAL/CAN.
public class Dummy extends Subsystem {
	private static Dummy instance = new Dummy();
	public static Dummy getInstance() { return instance; }
	private DummyIO periodic;

	public enum State {
		kRunning,
		kStopped
	}

	public void setState(State state) {
		if (Constants.DEBUG_LEVEL > 0) {
			System.out.println("Set state to " + state);
		}
		periodic.state = state;
	}

	public int getCount() {
		return periodic.count;
	}

	public Dummy() {
		periodic = new DummyIO();
		periodic.state = State.kRunning;
	}

	/**
	 * Updates all periodic variables and sensors
	 */
	public void readPeriodicInputs() {}

	/**
	 * Writes the periodic outputs to actuators (motors and ect...)
	 */
	public void writePeriodicOutputs() {
		if (periodic.state == State.kRunning) {
			periodic.count++;
		}
	}

	/**
	 * Outputs all logging information to the SmartDashboard
	 */
	public void outputTelemetry() {
		SmartDashboard.putNumber("Dummy count", periodic.count);
		SmartDashboard.putNumber("Debug level", Constants.DEBUG_LEVEL);
	}

	/**
	 * Called to reset and configure the subsystem
	 */
	public void reset() {}

	public class DummyIO extends PeriodicIO {
		public int count = 0;
		public State state;
	}

	public LogData getLogger() {
		return periodic;
	}
}
