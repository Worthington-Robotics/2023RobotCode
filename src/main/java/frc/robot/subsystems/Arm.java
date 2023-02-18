package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.lib.loops.ILooper;
import frc.lib.loops.Loop;
import frc.lib.util.Util;
import frc.lib.util.HIDHelper;


public class Arm extends Subsystem {
	TalonFX extensionMotor, turretMotor, armMasterMotor, armSlaveMotor;
	DoubleSolenoid grabber;
	DigitalInput bottomLimitSwitch;

	private static Arm instance = new Arm();
	public static Arm getInstance() { return instance; }
	private ArmIO periodic;

	public Arm() {
		extensionMotor = new TalonFX(Constants.ARM_EXTENSION_ID, "Default Name");
		turretMotor = new TalonFX(Constants.ARM_TURRET_ID, "Default Name");
		armMasterMotor = new TalonFX(Constants.ARM_ARM_M_ID, "Default Name");
		armSlaveMotor = new TalonFX(Constants.ARM_ARM_S_ID, "Default Name");

		grabber = new DoubleSolenoid(
			Constants.CTRE_PCM_ID,
			PneumaticsModuleType.CTREPCM,
			Constants.ARM_GRABBER_FWD_CHANNEL,
			Constants.ARM_GRABBER_REV_CHANNEL
		);

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
		public double turretPower;
		public double pivotPower;
		public double extensionPower;
		public double rawTurretPower; //take in values from HID helper
		public double rawPivotPower; //take in values from HID helper
		public double rawExtensionPower; //take in values from HID helper
		public DoubleSolenoid.Value grabberEngaged = Value.kReverse;

		// Encoder Values
		public double pivotEncoder;
		public double lengthEncoder;
		public double turretEncoder;

		// Actual Values
		public double armDegree; // Assumes arm starts at 0 degrees (all the way down)
		public double armLength; // Assumes arm starts at 0 inches of extension 
		public double turretDegree; // Assumes arm starts at 0 degrees (centered)

		// Desired values
		public double desiredPivotDegree; // from 0 to the maximum extension of the arm 
		public double desiredArmLength; // inches
		public double desiredTurretDegree; // scale of -135 to 135
		public double desiredPivotEncoder;
		public double desiredArmLengthEncoder;
		public double desiredTurretDegreeEncoder;


		// Error Values
		public double pivotError;
		public double lengthError;
		public double turretError;

		//state
		public ArmMode currentMode;

		//arbitrary feed forward
		public double arbitraryFeedForward;
	}

	public void readPeriodicInputs() {
		periodic.pivotEncoder = armMasterMotor.getSelectedSensorPosition();
		periodic.lengthEncoder = extensionMotor.getSelectedSensorPosition();
		periodic.turretDegree = turretMotor.getSelectedSensorPosition();

		periodic.armDegree = (periodic.pivotEncoder / Constants.ENCODER_PER_DEGREE) + 20.0;
		periodic.armLength = periodic.lengthEncoder / Constants.ENCODER_PER_INCH;
		periodic.turretDegree = periodic.pivotEncoder / Constants.ENCODER_PER_DEGREE;

		periodic.rawPivotPower = HIDHelper.getAxisMapped(Constants.SECOND.getRawAxis(3), 1, 0);
		periodic.rawExtensionPower = HIDHelper.getAxisMapped(Constants.SECOND.getRawAxis(1), .5, -.5);
		periodic.rawTurretPower = HIDHelper.getAxisMapped(Constants.SECOND.getRawAxis(0), -.25, .25);

	}

	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(new Loop(){

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
						periodic.desiredPivotEncoder = periodic.rawPivotPower; //multiply by constant
						armAnglePID();
						periodic.desiredArmLength = periodic.rawExtensionPower; //multiply by constant
						armExtensionPID();
						periodic.desiredTurretDegree = periodic.rawTurretPower; //multiply by sontant
						turretAnglePID();
					case CLOSED_LOOP:
						armAnglePID();
						armExtensionPID();
						turretAnglePID();
				}
			}

			@Override
			public void onStop(double timestamp) {
				reset();
				
			}
			
		});
		
	}

	public void writePeriodicOutputs() {
		extensionMotor.set(ControlMode.PercentOutput, periodic.extensionPower);
		turretMotor.set(ControlMode.PercentOutput, periodic.turretPower);
		
		armMasterMotor.set(ControlMode.Position, periodic.desiredPivotEncoder, DemandType.ArbitraryFeedForward, periodic.pivotPower);
		armSlaveMotor.set(ControlMode.Follower, Constants.ARM_ARM_M_ID);
	}

	public void outputTelemetry() {
		SmartDashboard.putNumber("Arm/turretPower", periodic.turretPower);
		SmartDashboard.putNumber("Arm/armPower", periodic.pivotPower);
		SmartDashboard.putNumber("Arm/extensionPower", periodic.extensionPower);
		SmartDashboard.putString("Arm/grabberEngaged", periodic.grabberEngaged.toString());
		SmartDashboard.putNumber("Arm/arbitaryFeed", periodic.arbitraryFeedForward);
	}

	public void reset() {
		resetEncoders();	
		periodic.desiredPivotDegree = 0.0; // from 0 to the maximum extension of the arm 
		periodic.desiredArmLength = 0.0; // inches
		periodic.desiredTurretDegree = 0.0; // scale of -135 to 135
		
		periodic.pivotPower = 0.0;
		periodic.extensionPower = 0.0;
		periodic.turretPower = 0.0;
		periodic.currentMode = ArmMode.OPEN_LOOP;
	}
	
	// Getters

	public double getPivotError(){
		return periodic.pivotError;
	}

	public double getLengthError(){
		return periodic.lengthError;
	}

	public double getTurretError(){
		return periodic.turretError;
	}

	// for closed loop
	public void setDesiredPivot(double theta) {
		periodic.desiredPivotDegree = theta;
		periodic.desiredPivotEncoder = theta * Constants.ENCODER_PER_DEGREE;
		periodic.pivotError = periodic.desiredPivotDegree - periodic.armDegree; 
		periodic.currentMode = ArmMode.CLOSED_LOOP;
	}

	public void setDesiredTurretDegree(double theta) {
		periodic.desiredTurretDegree = theta;
		periodic.turretError = periodic.desiredTurretDegree - periodic.turretDegree;
		periodic.currentMode = ArmMode.CLOSED_LOOP;
	}

	public void setDesiredLength(double length) {
		periodic.desiredArmLength = length;
		periodic.lengthError = periodic.desiredArmLength - periodic.armLength;
		periodic.currentMode = ArmMode.CLOSED_LOOP; 
	}
	 
	// Subsystem Exclusive Utilities - Could be private
		
	// public double safetyLimit(double power, double position, double warningRange, double min, double max) {		
	// 	// warnings - slows down motors when get close to limit
	// 	if(position < (min + warningRange) && power < 0.0){ // If the arm is moving in the direction of its minimum value and is approaching its warning point
	// 		return -0.1; // set to minimum motor speed, probably should be changed as different for each part of arm
	// 	}
	// 	else if(position > (max - warningRange) && power > 0.0){
	// 		return 0.1;
	// 	}
	// 	// Safety shutoff - Disable motors
	// 	else if(position <= min && power < 0.0){
	// 		return 0.0;
	// 	}
	// 	else if(position >= max && power > 0.0){
	// 		return 0.0;
	// 	}
	// 	// Fine
	// 	else{
	// 		return power;
	// 	}
	// }

	public void resetEncoders() {
		extensionMotor.setSelectedSensorPosition(0);
		turretMotor.setSelectedSensorPosition(0);
		armMasterMotor.setSelectedSensorPosition(0);
		armSlaveMotor.setSelectedSensorPosition(0);
	}

	// MainLoop Functions (Mostly PID) - Could be private

	public void armAnglePID() {
		double leverLengthCoeff = periodic.armLength * Constants.ARM_EXTENSION_KP;
		periodic.pivotPower = Math.sin(periodic.armDegree) * Constants.ARM_PIVOT_KP * leverLengthCoeff;
		periodic.pivotPower = Util.clampSpeed(periodic.pivotPower, Constants.PIVOT_MIN_SPEED, Constants.PIVOT_MAX_SPEED);
		periodic.arbitraryFeedForward = Math.sin(periodic.armDegree) * Constants.ARM_PIVOT_KP * leverLengthCoeff;
	}

	public void turretAnglePID() {
		periodic.turretPower = periodic.turretError * Constants.ARM_EXTENSION_KP;
		periodic.turretPower = Util.clampSpeed(periodic.turretPower, Constants.TURRET_MIN_SPEED, Constants.TURRET_MAX_SPEED);
	}

	public void armExtensionPID() {
		periodic.extensionPower = periodic.lengthError * Constants.ARM_EXTENSION_KP;
		periodic.extensionPower = Util.clampSpeed(periodic.extensionPower, Constants.EXTENSION_MIN_SPEED, Constants.EXTENSION_MAX_SPEED);
		// periodic.extensionPower = safetyLimit(periodic.extensionPower, periodic.armLength, 
		// 	Constants.EXTENSION_WARNING_DISTANCE, Constants.MIN_ARM_LENGTH, Constants.MAX_ARM_LENGTH);
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

	public void setGrabber(DoubleSolenoid.Value value) {
		periodic.grabberEngaged = value;
	}
	// public void moveArm(double currentDegreeEncoder, double extensionEncoder, double desiredArmDegree){
	// 	periodic.armDegree = 20.0 + (currentDegreeEncoder / Constants.ENCODER_PER_DEGREE);
	// 	periodic.armLength = 20.0 + (extensionEncoder / Constants.ENCODER_PER_INCH);
	// 	periodic.desiredArmDegree = desiredArmDegree;

	// 	double leverVal = periodic.armLength * Constants.LEVER_LENGTH_KP;
	// 	double normalizedArmPower = Math.sin(periodic.armDegree) * Constants.ARM_POWER_KP * leverVal;
	// 	double degreeError = periodic.armDegree - desiredArmDegree;

	// 	periodic.armPower = normalizedArmPower * degreeError * (1.0 / 60.0);
	// }

	// Logging

	public LogData getLogger() {
		return periodic;
	}
}
