package frc.lib.control;

import edu.wpi.first.wpilibj.Timer;

/**
 * Utility class that checks if a value is within an error margin.
 * It can also optionally check if the value is within the margin for
 * a minimum amount of time
 */
public class ErrorChecker {
	private double errorMargin;
	private double minTime;
	private double lastTime;

	public ErrorChecker(double errorMargin) {
		this(errorMargin, 0.0);
	}

	/**
	 * Initializes the error checker with a set minimum time.
	 * The checker will not succeed unless the error has been within
	 * the margin 
	 */
	public ErrorChecker(double errorMargin, double minTime) {
		this.errorMargin = errorMargin;
		this.minTime = minTime;
		lastTime = Timer.getFPGATimestamp();
	}

	// Check if all the conditions have been met with a provided error
	public boolean check(double error) {
		if (Math.abs(error) < errorMargin) {
			return (Timer.getFPGATimestamp() - lastTime) > minTime;
		} else {
			lastTime = Timer.getFPGATimestamp();
			return false;
		}
	}
}
