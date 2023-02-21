package frc.robot;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;
import frc.robot.subsystems.DriveTrain;
import frc.lib.util.Util;

public class DriveTests {
	@Test
	public void testDriveClamping() {
		final double min = 1.0;
		final double max = 5.0;
		final double avg = (min + max) / 2;

		System.out.println("Testing a normal number");
		assertTrue(Util.epsilonEquals(DriveTrain.clampDriveSpeed(avg, min, max), avg));

		System.out.println("Testing a number under the minimum");
		assertTrue(Util.epsilonEquals(DriveTrain.clampDriveSpeed(min / 2, min, max), min));

		System.out.println("Testing a number over the maximum");
		assertTrue(Util.epsilonEquals(DriveTrain.clampDriveSpeed(max + 100, min, max), max));

		System.out.println("Testing a negative number");
		assertTrue(Util.epsilonEquals(DriveTrain.clampDriveSpeed(-avg, min, max), -avg));
	}

	@Test
	public void testHeadingNormalization() {
		System.out.println("Testing a normal number");
		final double withinTest = DriveTrain.normalizeHeading(180);
		assertTrue((withinTest >= 0) && (withinTest <= 360));

		System.out.println("Testing a large number");
		final double largeTest = DriveTrain.normalizeHeading(Double.MAX_VALUE / 2);
		assertTrue((largeTest >= 0) && (largeTest <= 360));

		System.out.println("Testing a large negative number");
		final double negativeTest = DriveTrain.normalizeHeading(Double.MIN_VALUE / 2);
		assertTrue((negativeTest >= 0) && (negativeTest <= 360));

		System.out.println("Testing zero");
		final double zeroTest = DriveTrain.normalizeHeading(-0.0d);
		assertTrue((zeroTest >= 0) && (zeroTest <= 360));
	}

	@Test
	public void testHeadingErrorNormalization() {
		System.out.println("Testing a normal number");
		final double withinTest = DriveTrain.normalizeHeadingError(75);
		assertTrue((withinTest >= -180) && (withinTest <= 180));

		System.out.println("Testing a number greater than 180");
		final double greaterTest = DriveTrain.normalizeHeadingError(355);
		assertTrue((greaterTest >= -180) && (greaterTest <= 180));

		System.out.println("Testing a number less than -180");
		final double lessTest = DriveTrain.normalizeHeadingError(-280);
		assertTrue((lessTest >= -180) && (lessTest <= 180));

		System.out.println("Testing zero");
		final double zeroTest = DriveTrain.normalizeHeadingError(-0.0d);
		assertTrue((zeroTest >= -180) && (zeroTest <= 180));
	}
}
