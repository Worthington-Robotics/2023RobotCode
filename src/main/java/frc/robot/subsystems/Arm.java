package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.HIDHelper;
import frc.robot.Constants;
import frc.lib.loops.ILooper;
import frc.lib.loops.Loop;

public class Arm extends Subsystem {
	TalonFX extensionMotor, turretMotor, armMasterMotor, armSlaveMotor;

	private static Arm instance = new Arm();
	public static Arm getInstance() { return instance; }
	private ArmIO periodic;

	public Arm() {
		extensionMotor = new TalonFX(Constants.ARM_EXTENSION_ID, "Default Name");
		turretMotor = new TalonFX(Constants.ARM_TURRET_ID, "Default Name");
		armMasterMotor = new TalonFX(Constants.ARM_ARM_M_ID, "Default Name");
		armSlaveMotor = new TalonFX(Constants.ARM_ARM_S_ID, "Default Name");

		extensionMotor.setNeutralMode(NeutralMode.Brake);
		turretMotor.setNeutralMode(NeutralMode.Brake);
		armMasterMotor.setNeutralMode(NeutralMode.Brake);
		armSlaveMotor.setNeutralMode(NeutralMode.Brake);
		armSlaveMotor.setInverted(InvertType.FollowMaster);

		periodic = new ArmIO();
	}

	public enum ArmMode {
		OPEN_LOOP,
		OPEN_CLOSED_LOOP,
		CLOSED_LOOP
    }


	public class ArmIO extends PeriodicIO {
		// Power Values
		public double turretPower = 0;
		public double pivotPower = 0;
		public double extensionPower = 0;

		//values taken from HID helper
		public double rawTurretPower = 0;
		public double rawPivotPower = 0;
		public double rawExtensionPower = 0;

		// Encoder Values
		public double pivotEncoder;
		public double lengthEncoder;
		public double turretEncoder;

		// Actual Values
		public double pivotDegree; // Assumes arm starts at 25 degrees (all the way down)
		public double armLength; // Assumes arm starts at 0 inches of extension 
		public double turretDegree; // Assumes arm starts at 0 degrees (centered)

		// Desired values
		public double desiredPivotDegree; // from 25 to the maximum extension of the arm 
		public double desiredArmLength; // inches
		public double desiredTurretDegree; // scale of -135 to 135
		public double desiredPivotEncoder;
		public double desiredArmLengthEncoder;
		public double desiredTurretEncoder;

		// Error Values
		public double pivotEncoderError; //in ticks
		public double lengthEncoderError;
		public double turretEncoderError;

		// State
		public ArmMode currentMode = ArmMode.OPEN_LOOP;

		// Arbitrary feed forward
		public double arbitraryFeedForward;

		// Is pressed
		public boolean turretButtonIsPressed = false;
		public boolean extensionButtonIsPressed = false;
	}

	public void readPeriodicInputs() {
		periodic.pivotEncoder = armMasterMotor.getSelectedSensorPosition();
		periodic.lengthEncoder = extensionMotor.getSelectedSensorPosition();
		periodic.turretEncoder = turretMotor.getSelectedSensorPosition();

		periodic.pivotDegree = (periodic.pivotEncoder / Constants.PIVOT_ENCODER_PER_DEGREE) + 25.0;
		periodic.armLength = periodic.lengthEncoder / Constants.ENCODER_PER_INCH;
		periodic.turretDegree = periodic.turretEncoder / Constants.TURRET_ENCODER_PER_DEGREE;

		periodic.rawExtensionPower = HIDHelper.getAxisMapped(Constants.SECOND.getRawAxis(1), .5, -.5);
		periodic.rawTurretPower = HIDHelper.getAxisMapped(Constants.SECOND.getRawAxis(0), -.25, .25);
		periodic.rawPivotPower = HIDHelper.getAxisMapped(Constants.SECOND.getRawAxis(3), 1,0);
	}

	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(new Loop() {
			@Override
			public void onStart(double timestamp) {
				reset();
			}

			@Override
			public void onLoop(double timestamp) {
				switch (periodic.currentMode) {
					case OPEN_LOOP:
						setTurretPower(periodic.rawTurretPower);
						setPivotPower(periodic.rawPivotPower);
						setExtensionPower(periodic.rawExtensionPower);
						break;
					case OPEN_CLOSED_LOOP:
						periodic.desiredArmLengthEncoder = convertRawExtensionIntoEncoder(periodic.rawExtensionPower);
						periodic.desiredPivotEncoder = convertRawPivotIntoEncoder(periodic.rawPivotPower);
						periodic.desiredTurretEncoder = convertRawTurretIntoEncoder(periodic.rawTurretPower);
						
						periodic.desiredArmLength = periodic.desiredArmLengthEncoder  / Constants.ENCODER_PER_INCH;
						periodic.desiredPivotDegree = (periodic.desiredPivotEncoder / Constants.PIVOT_ENCODER_PER_DEGREE) + 25.0;
						periodic.desiredTurretDegree = periodic.desiredTurretEncoder / Constants.TURRET_ENCODER_PER_DEGREE;

						armAnglePID();
						armExtensionPID();
						turretAnglePID();
						break;
					case CLOSED_LOOP:
						armAnglePID();
						armExtensionPID();
						turretAnglePID();
						break;
				}
			}

			@Override
			public void onStop(double timestamp) {
				reset();
			}
		});
	}

	public void writePeriodicOutputs() {
		turretMotor.set(ControlMode.PercentOutput, periodic.turretPower);
		armMasterMotor.set(ControlMode.PercentOutput, periodic.pivotPower);
		armSlaveMotor.set(ControlMode.Follower, Constants.ARM_ARM_M_ID);
		
		if (periodic.currentMode == ArmMode.CLOSED_LOOP) {
			armMasterMotor.set(ControlMode.Position, periodic.desiredPivotEncoder);
			armSlaveMotor.set(ControlMode.Position, periodic.desiredPivotEncoder);
			extensionMotor.set(ControlMode.Position, periodic.desiredArmLengthEncoder);
		}
	}

	public void outputTelemetry() {
		SmartDashboard.putNumber("Arm/turretPower", periodic.turretPower);
		SmartDashboard.putNumber("Arm/rawTurretPower", periodic.rawTurretPower);
		SmartDashboard.putNumber("Arm/turretEncoder", periodic.turretEncoder);
		SmartDashboard.putNumber("Arm/turretDegree", periodic.turretDegree);

		SmartDashboard.putNumber("Arm/pivotPower", periodic.pivotPower);
		SmartDashboard.putNumber("Arm/rawPivotPower", periodic.rawPivotPower);
		SmartDashboard.putNumber("Arm/pivotEncoder", periodic.pivotEncoder);
		SmartDashboard.putNumber("Arm/pivotDegree", periodic.pivotDegree);
		SmartDashboard.putNumber("Arm/pivot encoder goal", periodic.desiredPivotEncoder);

		SmartDashboard.putNumber("Arm/extensionPower", periodic.extensionPower);
		SmartDashboard.putNumber("Arm/rawExtensionPower", periodic.rawExtensionPower);
		SmartDashboard.putNumber("Arm/lengthEncoder", periodic.lengthEncoder);
		SmartDashboard.putNumber("Arm/lengthOfExtension", periodic.armLength);
		SmartDashboard.putNumber("Arm/extension encoder goal", periodic.desiredArmLengthEncoder);

		SmartDashboard.putBoolean("Arm/turret button pressed", periodic.turretButtonIsPressed);
		SmartDashboard.putBoolean("Arm/extension button pressed", periodic.extensionButtonIsPressed);
	}
	

	public void reset() {
		periodic.currentMode = ArmMode.OPEN_CLOSED_LOOP;
		periodic.turretButtonIsPressed = false;
		periodic.extensionButtonIsPressed = false;
	}
	
	// Getters

	public double getTurretAngle() {
		return periodic.turretDegree;
	}
	public double getPivotError() {
		return periodic.pivotEncoderError;
	}

	public double getLengthError() {
		return periodic.lengthEncoderError;
	}

	public double getTurretError() {
		return periodic.turretEncoderError;
	}

	// For closed loop
	public void setDesiredPivot(double theta) {
		periodic.desiredPivotDegree = theta;
		periodic.desiredPivotEncoder = theta * Constants.PIVOT_ENCODER_PER_DEGREE;
		periodic.pivotEncoderError = periodic.desiredPivotEncoder - periodic.pivotEncoder;
	}

	public void setDesiredTurret(double theta) {
		periodic.desiredTurretDegree = theta;
		periodic.turretEncoderError = periodic.desiredTurretEncoder - periodic.turretEncoder;
	}

	public void setDesiredLength(double length) {
		periodic.desiredArmLength = length;
		periodic.lengthEncoderError = periodic.desiredArmLengthEncoder - periodic.lengthEncoder;
		periodic.desiredArmLengthEncoder = length * Constants.ENCODER_PER_INCH;
	}
	 

	public void resetEncoders() {
		extensionMotor.setSelectedSensorPosition(0);
		turretMotor.setSelectedSensorPosition(0);
		armMasterMotor.setSelectedSensorPosition(0);
		armSlaveMotor.setSelectedSensorPosition(0);
	}
	
	public double convertRawPivotIntoEncoder(double inputPower) {
		return inputPower * 30000;
	}

	public double convertRawExtensionIntoEncoder(double inputPower) {
		if (inputPower >= 0) {
			return inputPower * 125000;
		} else {
			return 0;
		}
	}

	public double convertRawTurretIntoEncoder(double inputPower) {
		return inputPower * 22000;
	}
	
	// MainLoop Functions (Mostly PID) - Could be private

	public void armAnglePID() {
		armMasterMotor.config_kP(0, Constants.ARM_PIVOT_KP);
		// double leverLengthCoeff = periodic.armLength * Constants.ARM_EXTENSION_KP;
		// periodic.pivotPower = Math.sin(periodic.armDegree) * Constants.ARM_PIVOT_KP * leverLengthCoeff;
		// periodic.pivotPower = Util.clampSpeed(periodic.pivotPower, Constants.PIVOT_MIN_SPEED, Constants.PIVOT_MAX_SPEED);
		// periodic.arbitraryFeedForward = Math.sin(periodic.armDegree) * Constants.ARM_PIVOT_KP * leverLengthCoeff;
	}

	public void turretAnglePID() {
		if (periodic.turretButtonIsPressed) {
			periodic.turretPower = periodic.rawTurretPower;
		}
		if (periodic.currentMode == ArmMode.CLOSED_LOOP) {
			turretMotor.config_kP(0, Constants.TURRET_KP);
		}
		//turretMotor.config_kP(0, Constants.TURRET_KP);
		// turretMotor.config_kP(0, getLengthError())
		// periodic.turretPower = periodic.turretError * Constants.TURRET_KP;
		// periodic.turretPower = Util.clampSpeed(periodic.turretPower, Constants.TURRET_MIN_SPEED, Constants.TURRET_MAX_SPEED);
	}

	public void armExtensionPID() {
		if (periodic.extensionButtonIsPressed) {
			extensionMotor.config_kP(0, Constants.ARM_EXTENSION_KP);
		}
		//periodic.extensionPower = periodic.lengthError * Constants.ARM_EXTENSION_KP;
		//periodic.extensionPower = Util.clampSpeed(periodic.extensionPower, Constants.EXTENSION_MIN_SPEED, Constants.EXTENSION_MAX_SPEED);
		// periodic.extensionPower = safetyLimit(periodic.extensionPower, periodic.armLength, 
		// 	Constants.EXTENSION_WARNING_DISTANCE, Constants.MIN_ARM_LENGTH, Constants.MAX_ARM_LENGTH);
	}

	public void setTurretButtonPressedTrue() {
		periodic.turretButtonIsPressed = true;
	}

	public void setExtensionButtonPressedTrue() {
		periodic.extensionButtonIsPressed = true;
	}

	public void setTurretButtonPressedFalse() {
		periodic.turretButtonIsPressed = false;
	}

	public void setExtensionButtonPressedFalse() {
		periodic.extensionButtonIsPressed = false;
	}

	public void setTurretPower(double power) {
		periodic.turretPower = power;
	}

	public void setPivotPower(double power) {
		periodic.pivotPower = power;
	}

	public void setExtensionPower(double power) {
		periodic.extensionPower = power;
	}

	public void setOpenLoop() {
		periodic.currentMode = ArmMode.OPEN_LOOP;
	}

	public void setOpenClosedLoop() {
		periodic.currentMode = ArmMode.OPEN_CLOSED_LOOP;
	}

	public void setClosedLoop() {
		periodic.currentMode = ArmMode.CLOSED_LOOP;
	}
	
	// Logging

	public LogData getLogger() {
		return periodic;
	}
}
