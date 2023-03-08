package frc.robot.subsystems;
import javax.lang.model.util.ElementScanner14;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.HIDHelper;
import frc.robot.Constants;
import frc.lib.loops.ILooper;
import frc.lib.loops.Loop;

public class Arm extends Subsystem {
	TalonFX extensionMotor, turretMotor, armMasterMotor;
	DoubleSolenoid pinSolenoid;

	private static Arm instance = new Arm();
	public static Arm getInstance() { return instance; }
	private ArmIO periodic = new ArmIO();

	public Arm() {
		extensionMotor = new TalonFX(Constants.ARM_EXTENSION_ID, "Default Name");
		turretMotor = new TalonFX(Constants.ARM_TURRET_ID, "Default Name");
		armMasterMotor = new TalonFX(Constants.ARM_ARM_M_ID, "Default Name");
		pinSolenoid = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 7, 6);
		extensionMotor.setNeutralMode(NeutralMode.Brake);
		turretMotor.setNeutralMode(NeutralMode.Brake);
		armMasterMotor.setNeutralMode(NeutralMode.Brake);
		reset();

	}

	public enum ArmMode {
		OPEN_LOOP,
		OPEN_CLOSED_LOOP,
		CLOSED_LOOP
    }

	public enum ArmPose {
		STOWN,
		UNSTOW,
		TRANSIT,
		INTAKE,
		SLIDE,
		CUBE_MID,
		CONE_MID,
		CUBE_HIGH,
		CONE_HIGH,
	}

	// Pivot, Extend, Wrist
	public static double[][] ArmPoses = {
		{0, 0, 0}, 
		{300000, 10000, 0},
		{113000, 3920, 11250},
		{88330, 49170, 27500},
		{200000, 0, 64000},
		{455200, 0, 4000},
		{515000, 63200, -3000},
		{563100, 122000, -15600},
		{626000, 125500, -3000}
	};

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

		// Desired values
<<<<<<< HEAD
		public double desiredPivotDegree;
		public double desiredArmLength;
		public double desiredTurretDegree; 
=======
		public double desiredPivotDegree; 
		public double desiredArmLength;
		public double desiredTurretDegree;
>>>>>>> 3f0bcf81d9c2615c6f2a78dc7ae6120af60cbf3c
		public double desiredPivotEncoder;
		public double desiredArmLengthEncoder;
		public double desiredTurretEncoder;

		// Error Values
		public double pivotEncoderError; //in ticks
		public double lengthEncoderError; //in ticks
		public double turretEncoderError; //in ticks

		// State
		public ArmMode currentMode = ArmMode.CLOSED_LOOP;
		public ArmPose currentPose = ArmPose.STOWN;

		// Arbitrary feed forward
		public double arbitraryFeedForward;

		// Is pressed
		public boolean turretButtonIsPressed = false;
		public boolean extensionButtonIsPressed = false;

		// Turret holds
		public double turretHoldValue = 0;
		public double extenHoldValue = 0;
		public boolean turretIsHolding = false;
		public boolean extendIsHolding = false;
	}

	public void readPeriodicInputs() {
		periodic.pivotEncoder = armMasterMotor.getSelectedSensorPosition();
		periodic.lengthEncoder = extensionMotor.getSelectedSensorPosition();
		periodic.turretEncoder = turretMotor.getSelectedSensorPosition();

		periodic.rawExtensionPower = HIDHelper.getAxisMapped(Constants.SECOND.getRawAxis(1), 1, -1);
		periodic.rawTurretPower = HIDHelper.getAxisMapped(Constants.SECOND.getRawAxis(0), -.25, .25);
		periodic.rawPivotPower = HIDHelper.getAxisMapped(Constants.SECOND.getRawAxis(3), 1,0);
	}

	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(new Loop() {
			@Override
			public void onStart(double timestamp) {
			}

			@Override
			public void onLoop(double timestamp) {
				switch (periodic.currentMode) {
					case OPEN_LOOP:
						break;
					case OPEN_CLOSED_LOOP:
						periodic.desiredArmLengthEncoder = convertRawExtensionIntoEncoder(periodic.rawExtensionPower);
						periodic.desiredPivotEncoder = convertRawPivotIntoEncoder(periodic.rawPivotPower);
						setTurretPower(periodic.rawTurretPower);

						periodic.pivotEncoderError = periodic.desiredPivotEncoder - periodic.pivotEncoder;
						periodic.lengthEncoderError = periodic.desiredArmLengthEncoder - periodic.lengthEncoder;
						periodic.turretEncoderError = periodic.desiredTurretEncoder - periodic.turretEncoder;
						break;
					case CLOSED_LOOP:
						periodic.desiredPivotEncoder = Arm.ArmPoses[periodic.currentPose.ordinal()][0];
						periodic.desiredArmLengthEncoder = Arm.ArmPoses[periodic.currentPose.ordinal()][1] + (periodic.rawExtensionPower * 10000);
						setTurretPower(periodic.rawTurretPower);

						periodic.pivotEncoderError = periodic.desiredPivotEncoder - periodic.pivotEncoder;
						periodic.lengthEncoderError = periodic.desiredArmLengthEncoder - periodic.lengthEncoder;
						periodic.turretEncoderError = periodic.desiredTurretEncoder - periodic.turretEncoder;
						break;
				}
			}

			@Override
			public void onStop(double timestamp) {
			}
		});
	}

	public void writePeriodicOutputs() {
		if(periodic.currentMode == ArmMode.OPEN_LOOP){
			turretMotor.set(ControlMode.PercentOutput, periodic.rawTurretPower);
			armMasterMotor.set(ControlMode.PercentOutput, periodic.rawPivotPower - .25);
			extensionMotor.set(ControlMode.PercentOutput, periodic.rawExtensionPower);
		} else {
			armMasterMotor.set(ControlMode.Position, periodic.desiredPivotEncoder);
			if(!periodic.turretIsHolding) {
				turretMotor.set(ControlMode.PercentOutput, periodic.rawTurretPower);
			} else {
				turretMotor.set(ControlMode.Position, periodic.turretHoldValue);
			}
			if(!periodic.extendIsHolding) {
				extensionMotor.set(ControlMode.Position, periodic.desiredArmLengthEncoder);
			} else {
				extensionMotor.set(ControlMode.Position, periodic.extenHoldValue);
			}
		}
	}

	public void outputTelemetry() {
		SmartDashboard.putNumber("Arm/encoder/turret", periodic.turretEncoder);
		SmartDashboard.putNumber("Arm/encoder/pivot", periodic.pivotEncoder);
		SmartDashboard.putNumber("Arm/encoder/extend", periodic.lengthEncoder);
		
		SmartDashboard.putNumber("Arm/encoder/turret-D", periodic.turretEncoder);
		SmartDashboard.putNumber("Arm/encoder/pivot-D", periodic.desiredPivotEncoder);
		SmartDashboard.putNumber("Arm/encoder/extend-D", periodic.desiredArmLengthEncoder);

		SmartDashboard.putString("Arm/curMode", periodic.currentMode.toString());
		SmartDashboard.putString("Arm/curPose", periodic.currentPose.toString());

		SmartDashboard.putNumber("Arm/turretHoldVal", periodic.turretHoldValue);
		SmartDashboard.putBoolean("Arm/turretIsHeld", periodic.turretIsHolding);
		SmartDashboard.putBoolean("Arm/turret button pressed", periodic.turretButtonIsPressed);
		SmartDashboard.putBoolean("Arm/extension button pressed", periodic.extensionButtonIsPressed);
	}

	public void reset() {
		periodic.currentMode = ArmMode.CLOSED_LOOP;
		periodic.turretButtonIsPressed = false;
		periodic.extensionButtonIsPressed = false;
		pinSolenoid.set(Value.kForward);
		configPID();
		resetEncoders();
	}
	
	

	public void clearPin() {
		pinSolenoid.set(Value.kReverse);
	}

	public void setPin() {
		pinSolenoid.set(Value.kForward);
	}

	public double getTurretEncoder() {
		return periodic.turretEncoder;
	}

	public double getExtendEncoder() {
		return periodic.lengthEncoder;
	}

<<<<<<< HEAD
	public double getPivotEncoder() {
		return periodic.pivotEncoder;
	}

	public double getTurretEncoderError() {
		return periodic.turretEncoderError;
	}

	public double getPivotEncoderError() {
		return periodic.pivotEncoderError;
	}

	public double getExtendEncoderError() {
		return periodic.lengthEncoderError;
	}


	public void setDesiredTurret(double thetaEncoder) {
		periodic.desiredTurretEncoder = thetaEncoder;
	}
=======
>>>>>>> 3f0bcf81d9c2615c6f2a78dc7ae6120af60cbf3c
	 
	public void setDisabledLimitSwtich() {
		turretMotor.overrideLimitSwitchesEnable(true);
	}

	public void setEnabledLimitSwtich() {
		turretMotor.overrideLimitSwitchesEnable(false);
	}

	public void resetEncoders() {
		extensionMotor.setSelectedSensorPosition(0);
		turretMotor.setSelectedSensorPosition(0);
		armMasterMotor.setSelectedSensorPosition(0);
	}


	public void configPID(){
		armMasterMotor.config_kP(0, Constants.ARM_PIVOT_KP);
		armMasterMotor.config_kI(0, 0);
		armMasterMotor.config_kD(0, 0);
		armMasterMotor.config_kF(0, 0);

		extensionMotor.config_kP(0,Constants.ARM_EXTENSION_KP);
		extensionMotor.config_kI(0,0);
		extensionMotor.config_kD(0,0);
		extensionMotor.config_kF(0,0);

		turretMotor.config_kP(0,Constants.TURRET_KP);
		turretMotor.config_kI(0,0);
		turretMotor.config_kD(0,Constants.TURRET_KD);
		turretMotor.config_kF(0,0);

	}
	
	public double convertRawPivotIntoEncoder(double inputPower) {
		return inputPower * 700000;
	}

	public double convertRawExtensionIntoEncoder(double inputPower) {
		if (inputPower >= 0) {
			return inputPower * 123000;
		} else {
			return 0;
		}
	}

	public double convertRawTurretIntoEncoder(double inputPower) {
		return inputPower * 22000;
	}
	
	// MainLoop Functions (Mostly PID) - Could be private



	public void setTurretPower(double power) {
		periodic.turretPower = power;
	}

<<<<<<< HEAD
	public void setPivotPower(double power) {
		periodic.pivotPower = power;
	}

	public void setExtensionPower(double power) {
		periodic.extensionPower = power;
	}

=======
>>>>>>> 3f0bcf81d9c2615c6f2a78dc7ae6120af60cbf3c
	public void cycleMode() {
		int nState = periodic.currentMode.ordinal() + 1;
		periodic.currentMode = ArmMode.values()[nState % 3];
	}

	public ArmMode getMode() {
		return periodic.currentMode;
	}

	public void setPose(ArmPose pose) {
		periodic.currentPose = pose;
	}

	public ArmPose getPose() {
		return periodic.currentPose;
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

	public void turretHoldLock(boolean enable, double Tlock) {
			periodic.turretHoldValue = Tlock;
			periodic.turretIsHolding = enable;
	}

	public void extendHoldLock(boolean enable, double Elock) {
		periodic.extenHoldValue = Elock;
		periodic.extendIsHolding = enable;
	}
	
}
