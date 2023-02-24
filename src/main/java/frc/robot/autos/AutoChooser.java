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
		None,
		One,
		Two,
		Test
	}

	// The currently chosen autonomous routine
	private AutoType chosen;

	private AutoChooser() {
		change(AutoType.Two);	
	}

	// Run the currently selected autonomous
	public void run() {
		switch (chosen) {
			case None:
				DebugLogger.getInstance().debugError(DebugLevel.kNone, "No autonomous is currently selected!");
				break;
			case One:
				StateMachine.getInstance().runMachine(new AutoOne());
				break;
			case Two:
				StateMachine.getInstance().runMachine(new AutoTwo());
				break;
			case Test:
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

	public void change(String name) {
		for (AutoType sel : AutoType.values()) {
			if (sel.toString().equalsIgnoreCase(name)) {
				chosen = sel;
				return;
			}
		}
		chosen = AutoType.None;
	}

	public void printList() {
		String[] list = new String[AutoType.values().length];
		for (int i = 0; i < AutoType.values().length; i++) {
			list[i] = AutoType.values()[i].toString();
		}
		SmartDashboard.putStringArray("Auto List", list);
	}

	public void run_from_selection() {
		final String selection = SmartDashboard.getString("Auto Selector", "None");
		change(selection);
		run();
	}

	// Logs current auto to telemetry
	public void logAuto() {
		SmartDashboard.putString("Auto/Current", chosen.toString());
	}
}