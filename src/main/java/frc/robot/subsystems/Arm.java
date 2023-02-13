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

public class Arm extends Subsystem {
	TalonFX extentionMotor, turretMotor, armMasterMotor, armSlaveMotor;
	DoubleSolenoid grabber;

	private static Arm instance = new Arm();
	public static Arm getInstance() { return instance; }
	private ArmIO periodic;

	public Arm() {
		extentionMotor = new TalonFX(Constants.ARM_EXTENTION_ID, "Default Name");
		turretMotor = new TalonFX(Constants.ARM_TURRET_ID, "Default Name");
		armMasterMotor = new TalonFX(Constants.ARM_ARM_M_ID, "Default Name");
		armSlaveMotor = new TalonFX(Constants.ARM_ARM_S_ID, "Default Name");
		grabber = new DoubleSolenoid(
			Constants.CTRE_PCM_ID,
			PneumaticsModuleType.CTREPCM,
			Constants.ARM_GRABBER_FWD_CHANNEL,
			Constants.ARM_GRABBER_REV_CHANNEL
		);

		extentionMotor.setNeutralMode(NeutralMode.Brake);
		turretMotor.setNeutralMode(NeutralMode.Brake);
		armMasterMotor.setNeutralMode(NeutralMode.Brake);
		armSlaveMotor.setNeutralMode(NeutralMode.Brake);
		armSlaveMotor.setInverted(InvertType.FollowMaster);

		periodic = new ArmIO();
	}

	public void readPeriodicInputs() {
		// Testing code for the arm
		// periodic.armPower = HIDHelper.getAxisMapped(Constants.SECOND.getRawAxis(3), 1, 0);
		// periodic.extentionPower = HIDHelper.getAxisMapped(Constants.SECOND.getRawAxis(1), .5, -.5);
		// periodic.turretPower = HIDHelper.getAxisMapped(Constants.SECOND.getRawAxis(0), -.25, .25);
	}

	public void writePeriodicOutputs() {
		extentionMotor.set(ControlMode.PercentOutput, periodic.extensionPower);
		turretMotor.set(ControlMode.PercentOutput, periodic.turretPower);
		
		armMasterMotor.set(ControlMode.PercentOutput, periodic.armPower);
		armSlaveMotor.set(ControlMode.Follower, Constants.ARM_ARM_M_ID);
	}

	public void outputTelemetry() {
		SmartDashboard.putNumber("Arm/turretPower", periodic.turretPower);
		SmartDashboard.putNumber("Arm/armPower", periodic.armPower);
		SmartDashboard.putNumber("Arm/extensionPower", periodic.extensionPower);
		SmartDashboard.putString("Arm/grabberEngaged", periodic.grabberEngaged.toString());
	}

	public void reset() {}

	public void setTurretPower(double power) {
		periodic.turretPower = power;
	}

	public void setArmPower(double power) {
		periodic.armPower = power;
	}

	public void setExtensionPower(double power) {
		periodic.extensionPower = power;
	}

	public class ArmIO extends PeriodicIO {
		public double turretPower = 0;
		public double armPower = 0;
		public double extensionPower = 0;
		public DoubleSolenoid.Value grabberEngaged = Value.kReverse;
	}

	public LogData getLogger() {
		return periodic;
	}
}
