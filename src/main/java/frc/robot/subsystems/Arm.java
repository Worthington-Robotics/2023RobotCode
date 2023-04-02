package frc.robot.subsystems;
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

//design pattern for caching periodic writes to avoid hammering the HAL/CAN.
public class Arm extends Subsystem {

<<<<<<< Updated upstream
	TalonFX extention, turret, armM, armS;
	DoubleSolenoid grabber;
=======
	public static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
	public static NetworkTableEntry ta = table.getEntry("ta");
	public static NetworkTableEntry tx = table.getEntry("tx");
	public static NetworkTableEntry tv = table.getEntry("tv");
    public static NetworkTableEntry snapshots = table.getEntry("snapshot");
    public static NetworkTableEntry pipeline = table.getEntry("pipeline");
	public static NetworkTableEntry aprilTag = table.getEntry("targetpose_cameraspace");
    public static double[] aprilTagDistances = aprilTag.getDoubleArray(new double[] {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
>>>>>>> Stashed changes

	private static Arm instance = new Arm();
	public static Arm getInstance() { return instance; }
	private ArmIO periodic;

	public Arm() {
		extention = new TalonFX(Constants.ARM_EXTENTION_ID, "Default Name");
		turret = new TalonFX(Constants.ARM_TURRET_ID, "Default Name");
		armM = new TalonFX(Constants.ARM_ARM_M_ID, "Default Name");
		armS = new TalonFX(Constants.ARM_ARM_S_ID, "Default Name");
		grabber = new DoubleSolenoid(Constants.CTRE_PCM_ID, PneumaticsModuleType.CTREPCM, Constants.ARM_GRABBER_FWD_CHANNEL, Constants.ARM_GRABBER_REV_CHANNEL);

		extention.setNeutralMode(NeutralMode.Brake);
		turret.setNeutralMode(NeutralMode.Brake);
		armM.setNeutralMode(NeutralMode.Brake);
		armS.setNeutralMode(NeutralMode.Brake);
		armS.setInverted(InvertType.FollowMaster);

		periodic = new ArmIO();
	}

	/**
	 * Updates all periodic variables and sensors
	 */
	public void readPeriodicInputs() {
		periodic.armPower = HIDHelper.getAxisMapped(Constants.SECOND.getRawAxis(3), 1, 0);
		periodic.extentionPower = HIDHelper.getAxisMapped(Constants.SECOND.getRawAxis(1), .5, -.5);
		periodic.turretPower = HIDHelper.getAxisMapped(Constants.SECOND.getRawAxis(0), -.25, .25);

<<<<<<< Updated upstream
		periodic.armEncoder = armM.getSelectedSensorPosition();
		periodic.extentionEncoder = extention.getSelectedSensorPosition();
		periodic.turretEncoder = turret.getSelectedSensorPosition();
	}

	/**
	 * Writes the periodic outputs to actuators (motors and ect...)
	 */
	public void writePeriodicOutputs() {
		extention.set(ControlMode.PercentOutput, periodic.extentionPower);
		turret.set(ControlMode.PercentOutput, periodic.turretPower);
		armM.set(ControlMode.PercentOutput, periodic.armPower);
		armS.set(ControlMode.Follower, Constants.ARM_ARM_M_ID);
	}

	/**
	 * Outputs all logging information to the SmartDashboard
	 */
	public void outputTelemetry() {
		SmartDashboard.putNumber("Arm/turretPower", periodic.turretPower);
		SmartDashboard.putNumber("Arm/armPower", periodic.armPower);
		SmartDashboard.putNumber("Arm/extentionPower", periodic.extentionPower);
		
		SmartDashboard.putNumber("Arm/turretEncoder", periodic.turretEncoder);
		SmartDashboard.putNumber("Arm/armEncoder", periodic.armEncoder);
		SmartDashboard.putNumber("Arm/extentionEncoder", periodic.extentionEncoder);

		SmartDashboard.putNumber("Arm/Stator/PivotCurrent", armM.getStatorCurrent());
		SmartDashboard.putNumber("Arm/Stator/ExtendCurrent", extention.getStatorCurrent());
		SmartDashboard.putNumber("Arm/Stator/TurretCurrent", turret.getStatorCurrent());
	}

	/**
	 * Called to reset and configure the subsystem
	 */
	public void reset() {}

	public void setClawOpen() {
		grabber.set(Value.kForward);
	}

	public void setClawClosed() {
		grabber.set(Value.kReverse);
	}
=======
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
		SHELF_END,
		MID_FRONT,
		HIGH_FRONT,
	}

	// Pivot, Extend, Wrist
	public static double[][] ArmPoses = {
		{0,0,0},
		{-109000, 0, 0},
		{-15384, 10000, 29290},
		{1775, 9100, 34290},
		{-78431, -56300, -42800},
		{-98372, -6800, -54000},
		{-270000, -77000, -123000},
		{-285000, -166000, -103000},
		{-355000, 6318, -113800},
		{-313000, 6318, -113800},
		{-189000, 0, -50500},
		{-229000, -140000, -63520},
	};
>>>>>>> Stashed changes

	public class ArmIO extends PeriodicIO {
		public double turretPower = 0;
		public double armPower = 0;
		public double extentionPower = 0;
		
		public double turretEncoder = 0;
		public double armEncoder = 0;
		public double extentionEncoder = 0;
		public DoubleSolenoid.Value grabberEngaged = Value.kReverse;
	}

<<<<<<< Updated upstream
=======
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
		if(Math.abs(periodic.turretRamp) < .05){
			periodic.turretRamp = .05 * (Math.signum(periodic.turretRamp));
		}
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

>>>>>>> Stashed changes
	public LogData getLogger() {
		return periodic;
	}
}
