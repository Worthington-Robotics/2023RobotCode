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
		None, TestAuto, TwoBarrierAuto, TwoBumpAuto, MiddleAuto, NoMoveAuto, PittsburghMiddleAuto, TestBarrierAuto, TestPathFollowerAuto
	}

	// The currently chosen autonomous routine
	private AutoType chosen;

	private AutoChooser() {
		change(AutoType.None);	
	}

	// Run the currently selected autonomous
	public void run() {
		switch (chosen) {
			case None:
				DebugLogger.getInstance().debugError(DebugLevel.kNone, "No autonomous is currently selected!");
				break;
			case TestAuto:
				StateMachine.getInstance().runMachine(new TestAuto());
				break;
			case TwoBarrierAuto:
				StateMachine.getInstance().runMachine(new TwoBarrierAuto());
				break;
			case TwoBumpAuto:
				StateMachine.getInstance().runMachine(new TwoBumpAuto());
				break;
			case MiddleAuto:
				StateMachine.getInstance().runMachine(new NewMiddleAuto());
				break;
			case NoMoveAuto:
				StateMachine.getInstance().runMachine(new NoMoveAuto());
				break;
			case PittsburghMiddleAuto:
				StateMachine.getInstance().runMachine(new PburghMiddleAuto());
				break;
			case TestBarrierAuto:
				StateMachine.getInstance().runMachine(new TestBarrierAuto());
				break;
			case TestPathFollowerAuto:
				StateMachine.getInstance().runMachine(new TestPathFollowerAuto());
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
