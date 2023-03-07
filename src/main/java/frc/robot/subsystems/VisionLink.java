package frc.robot.subsystems;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

//design pattern for caching periodic writes to avoid hammering the HAL/CAN.
public class VisionLink extends Subsystem {
	private static VisionLink instance = new VisionLink();
	public static VisionLink getInstance() { return instance; }
	private VisionIO periodic;

	public VisionLink() {
		periodic = new VisionIO();
	}

	public enum LimelightPipeline {
		Low,
		High
	}

	public enum SnapshotMode {
		Automatic,
		Manual
	}

	public class VisionIO extends PeriodicIO {
		public double turretOffset;
		public double angleOffset;
		public boolean hasTarget = false;
		public LimelightPipeline pipeline;
		public boolean takeSnapshot = false;
		public SnapshotMode snapshotMode = SnapshotMode.Automatic;
		public double lastSnapshotTime = 0.0;
	}

	public void readPeriodicInputs() {
		periodic.turretOffset = limelightGetNumber("tx");
		periodic.angleOffset = limelightGetNumber("ty");
		periodic.hasTarget = (limelightGetNumber("tv") != 0);

		if (periodic.snapshotMode == SnapshotMode.Automatic) {
			if ((Timer.getFPGATimestamp() - periodic.lastSnapshotTime) >= Constants.VISION_SNAPSHOT_INTERVAL) {
				periodic.lastSnapshotTime = Timer.getFPGATimestamp();
				periodic.takeSnapshot = true;
			}

			if (periodic.hasTarget) {
				periodic.takeSnapshot = true;
			}
		}
	}

	public void writePeriodicOutputs() {
		limelightSet("pipeline", getPipelineNumber(periodic.pipeline));
		if (periodic.takeSnapshot) {
			limelightSet("snapshot", 1);
			periodic.takeSnapshot = false;
		}
	}

	public void outputTelemetry() {
		SmartDashboard.putNumber("Vision/Turret Offset", periodic.turretOffset);
		SmartDashboard.putNumber("Vision/Angle Offset", periodic.angleOffset);
		SmartDashboard.putString("Vision/Limelight Pipeline", periodic.pipeline.toString());
	}

	// Pull data from the Limelight
	private double limelightGetNumber(String key) {
		return NetworkTableInstance.getDefault().getTable(Constants.LIMELIGHT_NETWORK_ID).getEntry(key).getDouble(0);
	}

	// Publish data to the Limelight
	private void limelightSet(String key, double value) {
		NetworkTableInstance.getDefault().getTable(Constants.LIMELIGHT_NETWORK_ID).getEntry(key).setNumber(value);
	}

	// Set the current pipeline
	public void setPipeline(LimelightPipeline pipeline) {
		periodic.pipeline = pipeline;	
	}

	// Gets the pipeline id from a pipeline
	public static int getPipelineNumber(LimelightPipeline pipeline) {
		int pipeline_number = 0;
		switch (pipeline) {
			case Low:
				pipeline_number = Constants.LIMELIGHT_LOW_PIPELINE;
				break;
			case High:
				pipeline_number = Constants.LIMELIGHT_HIGH_PIPELINE;
				break;
		}
		return pipeline_number;
	}

	// Take a snapshot with the limelight
	public void takeSnapshot() {
		periodic.takeSnapshot = true;
	}

	// Get the current turret offset
	public double getTurretOffset() {
		return periodic.turretOffset;
	}

	// Get the current angle offset
	public double getAngleOffset() {
		return periodic.angleOffset;
	}

	// Gets if the Limelight has a target
	public boolean hasTarget() {
		return periodic.hasTarget;
	}

	public void reset() {}

	public LogData getLogger() {
		return periodic;
	}
}