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

	TalonFX extention, turret, armM, armS;
	DoubleSolenoid grabber;

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
	}

	/**
	 * Called to reset and configure the subsystem
	 */
	public void reset() {}

	public class ArmIO extends PeriodicIO {
		public double turretPower = 0;
		public double armPower = 0;
		public double extentionPower = 0;
		public DoubleSolenoid.Value grabberEngaged = Value.kReverse;
	}

	public LogData getLogger() {
		return periodic;
	}
}
