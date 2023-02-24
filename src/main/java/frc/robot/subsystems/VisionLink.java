package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//design pattern for caching periodic writes to avoid hammering the HAL/CAN.
public class VisionLink extends Subsystem {
	private static VisionLink instance = new VisionLink();
	public static VisionLink getInstance() { return instance; }
	private VisionIO periodic;

	public VisionLink() {
		periodic = new VisionIO();
	}

	public void readPeriodicInputs() {
        //need to get turret angle from arm subsystem
        //also read smartdashboard for vision values
    }

	public void writePeriodicOutputs() {
        if (periodic.turretAngle >= 180) {
            periodic.turretAngle -= 0.1;
        } if (periodic.turretAngle <= 0) {
            periodic.turretAngle += 0.1;
        }
	}

    //add getters for vision values, so other subsystems dont need to get values from smartdash

	public void outputTelemetry() {
		SmartDashboard.putNumber("VisionData", periodic.turretAngle);
	}

	public void reset() {
        periodic.turretAngle = 0.0;
    }

	public class VisionIO extends PeriodicIO {
		public double turretAngle = 0;
	}

	public LogData getLogger() {
		return periodic;
	}
}
