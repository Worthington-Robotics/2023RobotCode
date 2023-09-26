package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.HIDHelper;
import frc.robot.Constants;
import frc.robot.subsystems.Subsystem;
import frc.lib.loops.ILooper;
import frc.lib.loops.Loop;

public class Arm extends Subsystem {
	TalonFX extensionMotor, armMasterMotor;

	private static Arm instance = new Arm();
	public static Arm getInstance() { return instance; }
	private ArmIO periodic = new ArmIO();
	private ArmKinematics kinematics = new ArmKinematics();
	Vector<N3> angles = kinematics.inverseSafe(ArmPoseNew.Preset.MID_CONE.getPose2d());
	private ArmVisualizer visualizer = new ArmVisualizer(angles);

	public Arm() {
		extensionMotor = new TalonFX(Constants.Arm.ARM_EXTENSION_ID, "Default Name");
		armMasterMotor = new TalonFX(Constants.Arm.ARM_ARM_M_ID, "Default Name");
		// extensionMotor.setNeutralMode(NeutralMode.Brake);
		// armMasterMotor.setNeutralMode(NeutralMode.Brake);
		extensionMotor.setNeutralMode(NeutralMode.Coast);
		armMasterMotor.setNeutralMode(NeutralMode.Coast);
		reset();
		extensionMotor.configVoltageCompSaturation(11);
		armMasterMotor.configVoltageCompSaturation(11); 
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
		SLIDE,
		INTAKE,
		CONE_UP,
		MID,
		HIGH,
		SHELF,
		HYBRID
	}

	// Pivot, Extend, Wrist
	public static double[][] ArmPoses = {
		{0,0,0},
		{-9623, 16110, 14270},
		{-21000, -1223, 18000},
		{-60700, -17100, -70200},
		{-98372, -5100, -54000},
		{-127000, -18728, -85700},
		{-138800, -117375, -85740},
		{-127000, 16110, -70430},
		{-9623, 16110, -9000},
	};

	public class ArmIO extends PeriodicIO {

		//values taken from HID helper
		public double rawPivotPower = 0;
		public double rawExtensionPower = 0;

		// Encoder Values
		public double pivotEncoder;
		public double lengthEncoder;

		// Desired values 
		public double desiredPivotEncoder;
		public double desiredArmLengthEncoder;

		// Error Values
		public double pivotEncoderError;
		public double lengthEncoderError;

		// State
		public ArmMode currentMode = ArmMode.CLOSED_LOOP;
		public ArmPose currentPose = ArmPose.ZERO;
		public boolean poseAccepted = false;

		// Turret and extend holds
		public double extenHoldValue = 0;
		public boolean extendIsHolding = false;

		public double pivotOffset = 0;
	}

	public void readPeriodicInputs() {
		periodic.pivotEncoder = armMasterMotor.getSelectedSensorPosition();
		periodic.lengthEncoder = extensionMotor.getSelectedSensorPosition();

		periodic.rawExtensionPower = HIDHelper.getAxisMapped(Constants.Joysticks.SECOND.getRawAxis(1), -.2, .2);
		periodic.rawPivotPower =  HIDHelper.getAxisMapped(Constants.Joysticks.SECOND.getRawAxis(3), -1,1);

		if(periodic.currentMode == ArmMode.CLOSED_LOOP){
			periodic.pivotOffset = HIDHelper.getAxisMapped(Constants.Joysticks.SECOND.getRawAxis(3), -1,0) * 20000;
		}
	}

	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(new Loop() {
			@Override
			public void onStart(double timestamp) {
			}

			@Override
			public void onLoop(double timestamp) {
				double x = Math.sin(timestamp);
				visualizer.update(kinematics.inverseSafe(new Pose2d(0.9+(0.3*x), 0.5 *x, new Rotation2d(0))));
				switch (periodic.currentMode) {
					case OPEN_LOOP:
						periodic.poseAccepted = true;
						break;
					case OPEN_CLOSED_LOOP:
						periodic.desiredArmLengthEncoder = convertRawExtensionIntoEncoder(periodic.rawExtensionPower);
						periodic.desiredPivotEncoder = convertRawPivotIntoEncoder(periodic.rawPivotPower);

						periodic.lengthEncoderError = periodic.desiredArmLengthEncoder - periodic.lengthEncoder;
						periodic.pivotEncoderError = periodic.desiredPivotEncoder - periodic.pivotEncoder;

						periodic.poseAccepted = (Math.abs(periodic.lengthEncoderError) < Constants.Arm.EXTENSION_ENCODER_ERROR_ACCEPTANCE) &&
						(Math.abs(periodic.pivotEncoderError) < Constants.Arm.PIVOT_ENCODER_ERROR_ACCEPTANCE);
						break;
					case CLOSED_LOOP:
						periodic.desiredPivotEncoder = Arm.ArmPoses[periodic.currentPose.ordinal()][0] + periodic.pivotOffset;
						periodic.desiredArmLengthEncoder = Arm.ArmPoses[periodic.currentPose.ordinal()][1];

						periodic.lengthEncoderError = periodic.desiredArmLengthEncoder - periodic.lengthEncoder;
						periodic.pivotEncoderError = periodic.desiredPivotEncoder - periodic.pivotEncoder;

						periodic.poseAccepted = (Math.abs(periodic.lengthEncoderError) < Constants.Arm.EXTENSION_ENCODER_ERROR_ACCEPTANCE) &&
						(Math.abs(periodic.pivotEncoderError) < Constants.Arm.PIVOT_ENCODER_ERROR_ACCEPTANCE) && 
						(Math.abs(Manipulator.getInstance().getWristEncoderError()) < Constants.Arm.WRIST_ENCODER_ERROR_ACCEPTANCE);
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
			armMasterMotor.set(ControlMode.Disabled, 0);
			extensionMotor.set(ControlMode.Disabled, 0);
		} else if (periodic.currentMode == ArmMode.OPEN_LOOP){
			armMasterMotor.set(ControlMode.PercentOutput, periodic.rawPivotPower);
			extensionMotor.set(ControlMode.PercentOutput, periodic.rawExtensionPower);
		} else {
			//set pivot
			if(Math.abs(periodic.desiredPivotEncoder) >= Math.abs(getPivotEncoder())){ //pivot is going up
				armMasterMotor.set(ControlMode.Position, periodic.desiredPivotEncoder);
			} else { //pivot is going down
				if(Math.abs(periodic.lengthEncoderError) <= 7000 && Math.abs(Manipulator.getInstance().getWristEncoderError()) <= 8500){
					armMasterMotor.set(ControlMode.Position, periodic.desiredPivotEncoder);
			    }
			}
			//set extension
			if(periodic.extendIsHolding && periodic.currentMode == ArmMode.OPEN_CLOSED_LOOP) {
				extensionMotor.set(ControlMode.Position, periodic.extenHoldValue);
			} else {
				if(Math.abs(periodic.desiredPivotEncoder) >= Math.abs(getPivotEncoder())){  //pivot going up
					if(Math.abs(periodic.pivotEncoderError) <= 5000){
						extensionMotor.set(ControlMode.Position, periodic.desiredArmLengthEncoder);
					}
				} else { //pivot is going down
					extensionMotor.set(ControlMode.Position, periodic.desiredArmLengthEncoder);
				}
			}
		}
	}

	public void outputTelemetry() {
		SmartDashboard.putNumber("Arm/Encoder/Pivot", periodic.pivotEncoder);
		SmartDashboard.putNumber("Arm/Encoder/Extend", periodic.lengthEncoder);

		SmartDashboard.putNumber("Arm/Open Loop Demand/Pivot", periodic.rawPivotPower);
		SmartDashboard.putNumber("Arm/Open Loop Demand/Extend", periodic.rawExtensionPower);
		
		SmartDashboard.putNumber("Arm/Close Loop Demand/Pivot", periodic.desiredPivotEncoder);
		SmartDashboard.putNumber("Arm/Close Loop Demand/Extend", periodic.desiredArmLengthEncoder);

		SmartDashboard.putString("Arm/Current Mode", periodic.currentMode.toString());
		SmartDashboard.putString("Arm/Current Pose", periodic.currentPose.toString());
		SmartDashboard.putBoolean("Arm/Error/Accepted", periodic.poseAccepted);


		SmartDashboard.putNumber("Arm/Error/Pivot Error", periodic.pivotEncoderError);
		SmartDashboard.putNumber("Arm/Error/Length Error", periodic.lengthEncoderError);
	}

	public void reset() {
		periodic.currentMode = ArmMode.CLOSED_LOOP;
		configPID();
		resetEncoders();
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

	public double getExtendEncoder() {
		return periodic.lengthEncoder;
	}

	public double getPivotEncoder() {
		return periodic.pivotEncoder;
	}

	public double getExtendEncoderError() {
		return periodic.lengthEncoderError;
	}

	public double getPivotEncoderError() {
		return periodic.pivotEncoderError;
	}

	public void resetEncoders() {
		extensionMotor.setSelectedSensorPosition(0);
		armMasterMotor.setSelectedSensorPosition(0);
	}

	public void resPivot() {
		periodic.pivotOffset = 0;
	}


	public void configPID(){
		armMasterMotor.config_kP(0, Constants.Arm.ARM_PIVOT_KP, 10);
		armMasterMotor.config_kI(0, 0, 10);
		armMasterMotor.config_kD(0, 0, 10);
		armMasterMotor.config_kF(0, 0, 10);

		extensionMotor.config_kP(0,Constants.Arm.ARM_EXTENSION_KP, 10);
		extensionMotor.config_kI(0,0, 10);
		extensionMotor.config_kD(0,0, 10);
		extensionMotor.config_kF(0,0, 10);
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

	public void setMode(ArmMode mode) {
		periodic.currentMode = mode;
		if(mode == ArmMode.DISABLED) {
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

	public void extendHoldLock(boolean enable, double Elock) {
		periodic.extenHoldValue = Elock;
		periodic.extendIsHolding = enable;
	}
	
}
