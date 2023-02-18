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
import frc.lib.loops.ILooper;
import frc.lib.loops.Loop;
import frc.lib.util.Util;


public class Arm extends Subsystem {
	TalonFX extensionMotor, turretMotor, armMasterMotor, armSlaveMotor;
	DoubleSolenoid grabber;

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
	public class ArmIO extends PeriodicIO {
		// Power Values
		public double turretPower = 0.0;
		public double pivotPower = 0.0;
		public double extensionPower = 0.0;
		public DoubleSolenoid.Value grabberEngaged = Value.kReverse;

		// Encoder Values
		public double pivotEncoder = 0.0;
		public double lengthEncoder = 0.0;
		public double turretEncoder = 0.0;

		// Actual Values
		public double armDegree = 0.0; // Assumes arm starts at 0 degrees (all the way down)
		public double armLength = 0.0; // Assumes arm starts at 0 inches of extension 
		public double turretDegree = 0.0; // Assumes arm starts at 0 degrees (centered)

		// Desired values
		public double desiredArmDegree = 0; // from 0 to the maximum extension of the arm 
		public double desiredArmLength = 0; // inches
		public double desiredTurretDegree = 0; // scale of -135 to 135


		// Error Values
		public double pivotError;
		public double lengthError;
		public double turretError;
	}

	public void readPeriodicInputs() {
		periodic.pivotEncoder = (armMasterMotor.getSelectedSensorPosition() + armSlaveMotor.getSelectedSensorPosition()) / 2;
		periodic.lengthEncoder = extensionMotor.getSelectedSensorPosition();
		periodic.turretDegree = turretMotor.getSelectedSensorPosition();

		periodic.armDegree = periodic.pivotEncoder / Constants.ENCODER_PER_DEGREE;
		periodic.armLength = periodic.lengthEncoder / Constants.ENCODER_PER_INCH;
		periodic.turretDegree = periodic.pivotEncoder / Constants.ENCODER_PER_DEGREE;
		// Testing code for the arm
		// periodic.armPower = HIDHelper.getAxisMapped(Constants.SECOND.getRawAxis(3), 1, 0);
		// periodic.extentionPower = HIDHelper.getAxisMapped(Constants.SECOND.getRawAxis(1), .5, -.5);
		// periodic.turretPower = HIDHelper.getAxisMapped(Constants.SECOND.getRawAxis(0), -.25, .25);
	}

	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(new Loop(){

			@Override
			public void onStart(double timestamp) {
				reset();
			}

			@Override
			public void onLoop(double timestamp) {
				armAnglePID();
				armExtensionPID();
				TurretAnglePID();
			}

			@Override
			public void onStop(double timestamp) {
				// TODO Auto-generated method stub
				
			}
			
		});
		
	}

	public void writePeriodicOutputs() {
		extensionMotor.set(ControlMode.PercentOutput, periodic.extensionPower);
		turretMotor.set(ControlMode.PercentOutput, periodic.turretPower);
		
		armMasterMotor.set(ControlMode.PercentOutput, periodic.pivotPower);
		armSlaveMotor.set(ControlMode.Follower, Constants.ARM_ARM_M_ID);
	}

	public void outputTelemetry() {
		SmartDashboard.putNumber("Arm/turretPower", periodic.turretPower);
		SmartDashboard.putNumber("Arm/armPower", periodic.pivotPower);
		SmartDashboard.putNumber("Arm/extensionPower", periodic.extensionPower);
		SmartDashboard.putString("Arm/grabberEngaged", periodic.grabberEngaged.toString());
	}

	public void reset() {
		resetEncoders();	
		periodic.desiredArmDegree = 0; // from 0 to the maximum extension of the arm 
		periodic.desiredArmLength = 0; // inches
		periodic.desiredTurretDegree = 0; // scale of -135 to 135

		// TODO: Move arm to 0s, than do above
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

	// Setters

	public void setDesiredPivot(double theta) {
		periodic.desiredArmDegree = theta;
		periodic.pivotError = periodic.desiredArmDegree - periodic.armDegree; 
	}

	public void setDesiredTurretDegree(double theta) {
		periodic.desiredTurretDegree = theta;
		periodic.turretError = periodic.desiredTurretDegree - periodic.turretDegree; 
	}

	public void setDesiredLength(double length) {
		periodic.desiredArmLength = length;
		periodic.armLength = periodic.desiredArmLength - periodic.armLength; 
	}
	 
	// Subsystem Exclusive Utilities - Could be private
		
	public double safetyLimit(double power, double position, double warning, double min, double max) {		
		// warnings - slows down motors when get close to limit
		if(position < (min+warning) && power < 0.0){ // If the arm is moving in the direction of its minimum value and is approaching its warning point
			return -0.1; // set to minimum motor speed, probably should be changed as different for each part of arm
		}
		else if(position > (max-warning) && power > 0.0){
			return 0.1;
		}
		// Safety shutoff - Disable motors
		else if(position <= min && power < 0.0){
			return 0.0;
		}
		else if(position >= max && power > 0.0){
			return 0.0;
		}
		// Fine
		else{
			return power;
		}
	}

	public void resetEncoders() {
		extensionMotor.setSelectedSensorPosition(0);
		turretMotor.setSelectedSensorPosition(0);
		armMasterMotor.setSelectedSensorPosition(0);
		armSlaveMotor.setSelectedSensorPosition(0);
	}

	public void limitcheck() {
		// TODO: Fill out
		// If limit switch pressed, reset pivot encoder and Arm must go up
	}

	// MainLoop Functions (Mostly PID) - Could be private

	public void armAnglePID() { // The power calculation assumes that the values are decreased when the motors go in reverse. TODO: Verify this behavior
		periodic.pivotError = periodic.desiredArmDegree - periodic.armDegree; 
		periodic.pivotPower = periodic.pivotError * Constants.ARM_PIVOT_KP;

		periodic.pivotPower = Util.clampSpeed(periodic.pivotPower, Constants.PIVOT_MIN_SPEED, Constants.PIVOT_MAX_SPEED);
		periodic.pivotPower = safetyLimit(periodic.pivotPower, periodic.armDegree, 
			Constants.PIVOT_WARNING_ANGLE, Constants.MIN_PIVOT, Constants.MAX_PIVOT); 
	}

	public void TurretAnglePID() {
		double adjAngle = periodic.turretDegree + 135.0; // TODO: Fix, temporary solution until safetyLimit() is fixed to work with negatives
		periodic.turretError = periodic.desiredTurretDegree - periodic.turretDegree;
		periodic.turretPower = periodic.turretError * Constants.ARM_EXTENSION_KP;

		periodic.turretPower = Util.clampSpeed(periodic.turretPower, Constants.TURRET_MIN_SPEED, Constants.TURRET_MAX_SPEED);
		periodic.turretPower = safetyLimit(periodic.turretPower, adjAngle, Constants.TURRET_WARNING_DISTANCE, 
			5.0, 265.0); // TODO: See comment on adjAngle
	}

	public void armExtensionPID() {
		periodic.lengthError = periodic.desiredArmLength - periodic.armLength;
		periodic.extensionPower = periodic.lengthError * Constants.ARM_EXTENSION_KP;

		periodic.extensionPower = Util.clampSpeed(periodic.extensionPower, Constants.EXTENSION_MIN_SPEED, Constants.EXTENSION_MAX_SPEED);
		periodic.extensionPower = safetyLimit(periodic.extensionPower, periodic.armLength, 
			Constants.EXTENSION_WARNING_DISTANCE, Constants.MIN_ARM_LENGTH, Constants.MAX_ARM_LENGTH);
	}
	
	// Old Code

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
