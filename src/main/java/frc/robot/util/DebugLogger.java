package frc.robot.util;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation;
import java.nio.file.Path;
import java.io.IOException;
import java.nio.file.Files;

// A class that will log information to the console only at specified debug levels
public class DebugLogger {
	private static DebugLogger instance = new DebugLogger();
	public static DebugLogger getInstance() { return instance; }

	public enum DebugLevel {
		kNone, // No debug prints or errors, useful for seeing errors caused by hardware
		kSome, // A few debug prints that show the general state of the robot, including errors
		kMost, // Pretty much everything important is printed
		kAll // Some less important things are printed as well
	}

	private DebugLevel debugLevel = DebugLevel.kNone;

	public DebugLogger() {
		Path debugPath = Filesystem.getDeployDirectory().toPath().resolve("debug.txt");
		try {
			String contents = new String(Files.readAllBytes(debugPath));
			// Casting from an int to an enum value
			final int levelInt = Integer.parseInt(contents);
			debugLevel = DebugLevel.values()[levelInt];
		} catch (IOException _err) {
			DriverStation.reportWarning("Failed to read debug level, setting to none", false);
			debugLevel = DebugLevel.kNone;
		}
	}

	// Returns the current debug level
	public DebugLevel getDebugLevel() {
		return debugLevel;
	}

	// Returns true if the current debug level is at least the specified level
	public boolean isDebugLevelAtLeast(DebugLevel level) {
		return (level.compareTo(debugLevel) >= 0);
	}

	// Print something to the console only at the specified debug level
	public void debugPrint(DebugLevel level, String message) {
		if (isDebugLevelAtLeast(level)) {
			System.out.println(message);
		}
	}

	// Report a warning to the driverstation only at the specified debug level
	public void debugWarn(DebugLevel level, String message) {
		if (isDebugLevelAtLeast(level)) {
			DriverStation.reportWarning(message, false);
		}
	}

	// Report an error to the driverstation only at the specified debug level
	public void debugError(DebugLevel level, String message) {
		if (isDebugLevelAtLeast(level)) {
			DriverStation.reportError(message, false);
		}
	}
}
