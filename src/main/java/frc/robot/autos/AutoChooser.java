package frc.robot.autos;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.statemachine.StateMachine;
import frc.robot.util.DebugLogger;
import frc.robot.util.DebugLogger.DebugLevel;

// Class that picks an autonomous routine to run based on user input
public class AutoChooser {
	private static AutoChooser instance = new AutoChooser();
	public static AutoChooser getInstance() { return instance; }
	
	public enum AutoType {
		kNone,
		kOne,
		kTwo,
		kTest
	}

	// The currently chosen autonomous routine
	private AutoType chosen;

	private AutoChooser() {
		change(AutoType.kTwo);
	}

	// Run the currently selected autonomous
	public void run() {
		switch (chosen) {
			case kNone:
				DebugLogger.getInstance().debugError(DebugLevel.kNone, "No autonomous is currently selected!");
				break;
			case kOne:
				StateMachine.getInstance().runMachine(new AutoOne());
				break;
			case kTwo:
				StateMachine.getInstance().runMachine(new AutoTwo());
				break;
			case kTest:
				StateMachine.getInstance().runMachine(new TestAuto());
				break;
		}
	}

	// Changes the autonomous routine to a specific one
	public void change(AutoType type) {
		chosen = type;
		DebugLogger.getInstance().debugPrint(DebugLevel.kSome, "Switched to autonomous " + type.toString());
		logAuto();
	}

	// Logs current auto to telemetry
	private void logAuto() {
		SmartDashboard.putString("Auto/Current Auto", chosen.toString());
	}
}
