package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.HIDHelper;
import frc.robot.Constants;
import frc.lib.loops.ILooper;
import frc.lib.loops.Loop;

public class Arm extends Subsystem {
	TalonFX extensionMotor, turretMotor, armMasterMotor;
	//DoubleSolenoid pinSolenoid;

	public static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-worbots");
	public static NetworkTableEntry ta = table.getEntry("ta");
	public static NetworkTableEntry tx = table.getEntry("tx");
	public static NetworkTableEntry tv = table.getEntry("tv");
    public static NetworkTableEntry snapshots = table.getEntry("snapshot");
    public static NetworkTableEntry pipeline = table.getEntry("pipeline");
	public static NetworkTableEntry aprilTag = table.getEntry("targetpose_cameraspace");
    public static double[] aprilTagDistances = aprilTag.getDoubleArray(new double[] {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

	private static Arm instance = new Arm();
	public static Arm getInstance() { return instance; }
	private ArmIO periodic = new ArmIO();

	public Arm() {
		extensionMotor = new TalonFX(Constants.ARM_EXTENSION_ID, "Default Name");
		turretMotor = new TalonFX(Constants.ARM_TURRET_ID, "Default Name");
		armMasterMotor = new TalonFX(Constants.ARM_ARM_M_ID, "Default Name");
		extensionMotor.setNeutralMode(NeutralMode.Brake);
		turretMotor.setNeutralMode(NeutralMode.Brake);
		armMasterMotor.setNeutralMode(NeutralMode.Brake);
		reset();
		extensionMotor.configVoltageCompSaturation(11);
		turretMotor.configVoltageCompSaturation(11);
		armMasterMotor.configVoltageCompSaturation(11);
		pipeline.setDouble(2); //set default to april tag pipeline
	}

	public enum ArmMode {
		DISABLED,
		OPEN_LOOP,
		OPEN_CLOSED_LOOP,
		CLOSED_LOOP
    }

	public enum ArmPose {
		ZERO,
		UNSTOW,
		TRANSIT,
		SLIDE,
		INTAKE,
		CONE_UP,
		MID,
		HIGH,
		SHELF_BEGIN,
		SHELF_END
	}

	// Pivot, Extend, Wrist
	public static double[][] ArmPoses = {
		{0,0,0},
		{-109000, 0, 0},
		{-15384, -10000, 29290},
		{1775, 9100, 29290},
		{-78431, -56300, -42800},
		{-98372, -6800, -54000},
		{-270000, -43260, -121400},
		{-280000, -178000, -106500},
		{-355000, 6318, -113800},
		{-313000, 6318, -113800}
	};

	public class ArmIO extends PeriodicIO {
		// turret power
		public double turretPower = 0;
		public double turretRamp = 0;

		//values taken from HID helper
		public double rawTurretPower = 0;
		public double rawPivotPower = 0;
		public double rawExtensionPower = 0;

		// Encoder Values
		public double pivotEncoder;
		public double lengthEncoder;
		public double turretEncoder;


		// Desired values 
		public double desiredPivotEncoder;
		public double desiredArmLengthEncoder;
		public double desiredTurretEncoder;

		// Error Values
		public double pivotEncoderError;
		public double lengthEncoderError;
		public double turretEncoderError;

		// State
		public ArmMode currentMode = ArmMode.CLOSED_LOOP;
		public ArmPose currentPose = ArmPose.ZERO;
		public boolean poseAccepted = false;

		// Turret and extend holds
		public double turretHoldValue = 0;
		public double extenHoldValue = 0;
		public boolean turretIsHolding = false;
		public boolean extendIsHolding = false;

		// Limelights
		public double LL_tx = 0;
		public double LL_tv = 0;
	}

	public void readPeriodicInputs() {
		periodic.pivotEncoder = armMasterMotor.getSelectedSensorPosition();
		periodic.lengthEncoder = extensionMotor.getSelectedSensorPosition();
		periodic.turretEncoder = turretMotor.getSelectedSensorPosition();

		periodic.rawExtensionPower = HIDHelper.getAxisMapped(Constants.SECOND.getRawAxis(1), -.2, .2);
		periodic.rawTurretPower = HIDHelper.getAxisMapped(Constants.SECOND.getRawAxis(0), .3, -.3);
		periodic.rawPivotPower =  HIDHelper.getAxisMapped(Constants.SECOND.getRawAxis(3), -1,1);

		periodic.LL_tv = tv.getDouble(0.0);
		periodic.LL_tx = tx.getDouble(0.0);
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
						periodic.poseAccepted = true;
						break;
					case OPEN_CLOSED_LOOP:
						periodic.desiredArmLengthEncoder = convertRawExtensionIntoEncoder(periodic.rawExtensionPower);
						periodic.desiredPivotEncoder = convertRawPivotIntoEncoder(periodic.rawPivotPower);
						setTurretPower(periodic.rawTurretPower);

						periodic.lengthEncoderError = periodic.desiredArmLengthEncoder - periodic.lengthEncoder;
						periodic.pivotEncoderError = periodic.desiredPivotEncoder - periodic.pivotEncoder;

						periodic.poseAccepted = (Math.abs(periodic.lengthEncoderError) < Constants.EXTENSION_ENCODER_ERROR_ACCEPTANCE) &&
						(Math.abs(periodic.pivotEncoderError) < Constants.PIVOT_ENCODER_ERROR_ACCEPTANCE) && 
						(Math.abs(periodic.turretEncoderError) < Constants.TURRET_ANGLE_ENCODER_ACCEPTANCE);
						break;
					case CLOSED_LOOP:
						periodic.desiredPivotEncoder = Arm.ArmPoses[periodic.currentPose.ordinal()][0];
						periodic.desiredArmLengthEncoder = Arm.ArmPoses[periodic.currentPose.ordinal()][1];
						setTurretPower(periodic.rawTurretPower);

						periodic.lengthEncoderError = periodic.desiredArmLengthEncoder - periodic.lengthEncoder;
						periodic.pivotEncoderError = periodic.desiredPivotEncoder - periodic.pivotEncoder;

						periodic.poseAccepted = (Math.abs(periodic.lengthEncoderError) < Constants.EXTENSION_ENCODER_ERROR_ACCEPTANCE) &&
						(Math.abs(periodic.pivotEncoderError) < Constants.PIVOT_ENCODER_ERROR_ACCEPTANCE) && 
						(Math.abs(Manipulator.getInstance().getWristEncoderError()) < Constants.WRIST_ENCODER_ERROR_ACCEPTANCE);
						break;
					default:
					break;
				}
			}

			@Override
			public void onStop(double timestamp) {
			}
		});
	}

	public void writePeriodicOutputs() {
		if(periodic.currentMode == ArmMode.DISABLED) {
			turretMotor.set(ControlMode.Disabled, 0);
			armMasterMotor.set(ControlMode.Disabled, 0);
			extensionMotor.set(ControlMode.Disabled, 0);
		} else if (periodic.currentMode == ArmMode.OPEN_LOOP){
			turretMotor.set(ControlMode.PercentOutput, periodic.rawTurretPower);
			armMasterMotor.set(ControlMode.PercentOutput, periodic.rawPivotPower);
			extensionMotor.set(ControlMode.PercentOutput, periodic.rawExtensionPower);
		} else {

			//set pivot
			if(Math.abs(periodic.desiredPivotEncoder) >= Math.abs(getPivotEncoder())){ //pivot is going up
				armMasterMotor.set(ControlMode.Position, periodic.desiredPivotEncoder);
			} else { //pivot is going down
				if(Math.abs(periodic.lengthEncoderError) <= 7000 && Math.abs(Manipulator.getInstance().getWristEncoderError()) <= 5000){
					armMasterMotor.set(ControlMode.Position, periodic.desiredPivotEncoder);
			    }
			}


			//set turret
			if(!periodic.turretIsHolding) {
				turretMotor.set(ControlMode.PercentOutput, periodic.rawTurretPower);
			} else {
				if(Math.abs(periodic.turretHoldValue - periodic.turretEncoder) < 10 * Constants.TURRET_TPD) {
					turretMotor.set(ControlMode.Position, periodic.turretHoldValue);
				} else {
					turretMotor.set(ControlMode.PercentOutput, Math.signum(periodic.turretHoldValue - periodic.turretEncoder) * Math.max(.12, Math.min(periodic.turretRamp, .4)));
				}
			}
			//set extension
			if(periodic.extendIsHolding && periodic.currentMode == ArmMode.OPEN_CLOSED_LOOP) {
				extensionMotor.set(ControlMode.Position, periodic.extenHoldValue);
			} else {
				if(Math.abs(periodic.pivotEncoder) >= 75000/*Math.abs(getPivotEncoder())*/){ 
					//if(Math.abs(periodic.pivotEncoderError) <= 10000){
						extensionMotor.set(ControlMode.Position, periodic.desiredArmLengthEncoder);
					//}
				} else { //pivot is going down
					extensionMotor.set(ControlMode.Position, periodic.desiredArmLengthEncoder);
				}
			}
		}
	}

	public void outputTelemetry() {

		SmartDashboard.putNumber("Arm/turretRamp", periodic.turretRamp);

		SmartDashboard.putNumber("Arm/encoder/turret", periodic.turretEncoder);
		SmartDashboard.putNumber("Arm/encoder/pivot", periodic.pivotEncoder);
		SmartDashboard.putNumber("Arm/encoder/extend", periodic.lengthEncoder);

		SmartDashboard.putNumber("Arm/rawPower/pivot", periodic.rawPivotPower);
		SmartDashboard.putNumber("Arm/rawPower/extend", periodic.rawExtensionPower);
		
		SmartDashboard.putNumber("Arm/encoder/turret-D", periodic.turretHoldValue);
		SmartDashboard.putNumber("Arm/encoder/pivot-D", periodic.desiredPivotEncoder);
		SmartDashboard.putNumber("Arm/encoder/extend-D", periodic.desiredArmLengthEncoder);

		SmartDashboard.putString("Arm/curMode", periodic.currentMode.toString());
		SmartDashboard.putString("Arm/curPose", periodic.currentPose.toString());
		SmartDashboard.putBoolean("Arm/error/accpeted", periodic.poseAccepted);

		SmartDashboard.putNumber("Arm/turretHoldVal", periodic.turretHoldValue);
		SmartDashboard.putBoolean("Arm/turretIsHeld", periodic.turretIsHolding);

		SmartDashboard.putNumber("Arm/error/pivot error", periodic.pivotEncoderError);
		SmartDashboard.putNumber("Arm/error/length error", periodic.lengthEncoderError);

		SmartDashboard.putNumber("Limelight/tx", periodic.LL_tx);
		SmartDashboard.putNumber("Limelight/tv", periodic.LL_tv);
		SmartDashboard.putNumber("Limelight/ta", 1.0 / Math.sqrt(ta.getDouble(0)));
		
	}

	public void reset() {
		periodic.currentMode = ArmMode.CLOSED_LOOP;
		//pinSolenoid.set(Value.kForward);
		configPID();
		resetEncoders();
	}
	
	public void setTVComp(double volts) {
		turretMotor.configVoltageCompSaturation(volts);
	}

	public void incrRamp(double ramp) {
		periodic.turretRamp += ramp;
	}

	public void clearRamp() {
		periodic.turretRamp = 0;
	}

	public void correctExtension(double extensionChange) {
		periodic.desiredArmLengthEncoder += extensionChange;
	}

	public boolean getAccepted() {
		return periodic.poseAccepted;
	}

	public ArmMode getMode() {
		return periodic.currentMode;
	}

	public ArmPose getPose() {
		return periodic.currentPose;
	}

	public double getDesiredPivot() {
		return periodic.desiredPivotEncoder;
	}

	public double getDesiredExtension() {
		return periodic.desiredArmLengthEncoder;
	}

	public double getTurretEncoder() {
		return periodic.turretEncoder;
	}

	public double getExtendEncoder() {
		return periodic.lengthEncoder;
	}

	public double getPivotEncoder() {
		return periodic.pivotEncoder;
	}

	
	public double getTurretEncoderError() {
		return periodic.turretEncoderError;
	}

	public double getExtendEncoderError() {
		return periodic.lengthEncoderError;
	}

	public double getPivotEncoderError() {
		return periodic.pivotEncoderError;
	}


	public double[] getLLVals() {
		return new double[] {periodic.LL_tx, periodic.LL_tv};
	}
	 

	public void resetEncoders() {
		extensionMotor.setSelectedSensorPosition(0);
		turretMotor.setSelectedSensorPosition(0);
		armMasterMotor.setSelectedSensorPosition(0);
	}


	public void configPID(){
		armMasterMotor.config_kP(0, Constants.ARM_PIVOT_KP, 10);
		armMasterMotor.config_kI(0, 0, 10);
		armMasterMotor.config_kD(0, 0, 10);
		armMasterMotor.config_kF(0, 0, 10);

		extensionMotor.config_kP(0,Constants.ARM_EXTENSION_KP, 10);
		extensionMotor.config_kI(0,0, 10);
		extensionMotor.config_kD(0,0, 10);
		extensionMotor.config_kF(0,0, 10);

		turretMotor.config_kP(0,Constants.TURRET_KP, 10);
		turretMotor.config_kI(0,Constants.TURRET_KI, 10);
		turretMotor.config_kD(0,Constants.TURRET_KD, 10);
		turretMotor.config_kF(0,0, 10);

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
	

	public void setTurretPower(double power) {
		periodic.turretPower = power;
	}

	public void cycleMode() {
		int nState = periodic.currentMode.ordinal() + 1;
		periodic.currentMode = ArmMode.values()[nState % 3];
	}

	public void setMode(ArmMode mode) {
		periodic.currentMode = mode;
		if(mode == ArmMode.DISABLED) {
			turretMotor.set(ControlMode.Disabled, 0);
			armMasterMotor.set(ControlMode.Disabled, 0);
			extensionMotor.set(ControlMode.Disabled, 0);
		}
	}


	public void setPose(ArmPose pose) {
		periodic.currentPose = pose;
		periodic.poseAccepted = false;
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
