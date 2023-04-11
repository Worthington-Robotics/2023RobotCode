package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.Arm.ArmMode;
import frc.robot.subsystems.Arm.ArmPose;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.playingwithfusion.TimeOfFlight;

import frc.lib.util.HIDHelper;
import frc.lib.loops.ILooper;
import frc.lib.loops.Loop;


public class Manipulator extends Subsystem {
	private static Manipulator instance = new Manipulator();
	public static Manipulator getInstance() { return instance; }
	public ManipulatorIO periodic;
	private TalonFX wristMotor, intakeMotor;

	public enum State {
		OpenLoop,
		DriverControlled
	}

	public class ManipulatorIO extends PeriodicIO {
		double wristMotorPower;
		double intakeMotorPower;
		double rawWristMotorPower;
		double desiredWristEncoder;
		double wristEncoderError;
		double wristEncoder;
		double wristOffset;
		double prevDesiredWrist;
		double intakeMotorCurrent;
		boolean hasGamePiece;
		State state = State.DriverControlled;
		double startTime;
		double timeOffset;
	}

	public Manipulator() {
		periodic = new ManipulatorIO();
		wristMotor = new TalonFX(Constants.WRIST_MOTOR_ID, "Default Name");
		wristMotor.setNeutralMode(NeutralMode.Brake);
		intakeMotor = new TalonFX(Constants.INTAKE_MOTOR_ID, "Default Name");
		intakeMotor.setNeutralMode(NeutralMode.Brake);
		wristAnglePID();
	}

	public void readPeriodicInputs() {
		periodic.intakeMotorCurrent = intakeMotor.getSupplyCurrent(); //TODO: Add low current means that it doesn't have anything
		periodic.wristEncoder = wristMotor.getSelectedSensorPosition();
		double currentTime = Timer.getFPGATimestamp();
		if(periodic.state == State.DriverControlled) {
			if (periodic.intakeMotorCurrent > Constants.INTAKE_CURRENT_ACCEPTANCE) {
				periodic.hasGamePiece = true;
			}
			if (periodic.intakeMotorCurrent < 5 && periodic.intakeMotorCurrent > 0.5) {
				periodic.hasGamePiece = true;
			}
			if (periodic.intakeMotorPower < 0) {
				periodic.hasGamePiece = false;
			}
			if(periodic.intakeMotorCurrent == 0) {
				periodic.hasGamePiece = false;
			} 
			if (periodic.intakeMotorCurrent < 0.2) {
				periodic.hasGamePiece = false;
			}
		} 
		if(periodic.state == State.OpenLoop && (periodic.startTime + periodic.timeOffset) >= currentTime) {
			periodic.hasGamePiece = true;
		}
		if (periodic.state == State.OpenLoop && (periodic.startTime + periodic.timeOffset) < currentTime) {
			periodic.state = State.DriverControlled;
		}

	}

	public void writePeriodicOutputs() {
		if (!periodic.hasGamePiece) {
			intakeMotor.set(ControlMode.PercentOutput, periodic.intakeMotorPower);
		} else {
			intakeMotor.set(ControlMode.PercentOutput, Math.max(.1, periodic.intakeMotorPower));
		}
		if (Arm.getInstance().getMode().ordinal() < ArmMode.CLOSED_LOOP.ordinal() ) {
			wristMotor.set(ControlMode.Position, periodic.wristOffset);
		} else {
			if(Math.abs(Arm.getInstance().getDesiredPivot()) >= Math.abs(Arm.getInstance().getPivotEncoder())){//pivot going up
				if(Math.abs(Arm.getInstance().getPivotEncoderError()) < 10000){
					wristMotor.set(ControlMode.Position, periodic.desiredWristEncoder);
				}
			} else { //pivot going down
				//if(Math.abs(Arm.getInstance().getExtendEncoderError()) < 12000){
					wristMotor.set(ControlMode.Position, periodic.desiredWristEncoder);
				//}
			}
		}
	}

	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(new Loop() {
			@Override
			public void onStart(double timestamp) {
			}

			@Override
			public void onLoop(double timestamp) {
				switch (Arm.getInstance().getMode()) {
					case OPEN_LOOP:
						break;
					case OPEN_CLOSED_LOOP:
						periodic.desiredWristEncoder = convertRawWristPowerIntoEncoder(periodic.rawWristMotorPower);
						periodic.wristEncoderError = periodic.desiredWristEncoder - periodic.wristEncoder;
						break;
					case CLOSED_LOOP:
						periodic.desiredWristEncoder = Arm.ArmPoses[Arm.getInstance().getPose().ordinal()][2] + periodic.wristOffset;
						periodic.wristEncoderError = periodic.desiredWristEncoder - periodic.wristEncoder;
						break;
					case DISABLED:
						break;
				}
			}

			@Override
			public void onStop(double timestamp) {
			}
		});
	}

	public boolean isObject() {
		return periodic.hasGamePiece;
		// return (periodic.TimeOfFlightDistance < 200 && periodic.TimeOfFlightDistance != 0);
	}

	public void setGamePiece(boolean state) {
		periodic.hasGamePiece = state;
	}

	public void setDriverControlled() {
		periodic.state = State.DriverControlled;
	}

	public void setTimoutControlled() {
		periodic.state = State.OpenLoop;
	}

	public void setTimeOffset(double delay) {
		periodic.timeOffset = delay;
	}

	// Convert joystick values into motor powers
	public double convertRawWristPowerIntoEncoder(double inputPower) {
		return inputPower * 80000.0;
	}

	// Set the intake demand to the specified value
	public void setIntakePower(double power) {
			periodic.intakeMotorPower = power;
	}

	public void setWristPower(double power){
		periodic.wristMotorPower = power;
	}

	public void resWrist() {
		periodic.wristOffset = 0;
	}

	public void incWrist() {
		periodic.wristOffset += 5000;
	}

	public void decWrist() {
		periodic.wristOffset -= 5000;
	}

	// Getters

	public double getWristEncoder(){
		return periodic.wristEncoder;
	}

	public double getDesiredWrist() {
		return periodic.desiredWristEncoder;
	}

	public double getWristEncoderError(){
		return periodic.wristEncoderError;
	}

	public double getPrevDesiredWrist() {
		return periodic.prevDesiredWrist;
	}

	// PID
	
	public void wristAnglePID() {
		wristMotor.config_kP(0, Constants.WRIST_PIVOT_KP);
		wristMotor.config_kI(0,0);
		wristMotor.config_kD(0,0);
		wristMotor.config_kF(0,0);
	}

	public void setStartTime(double startTime) {
		periodic.startTime = startTime;
	}

	public void reset() { 
	}

	public void resetManipulatorEncoder(){
		wristMotor.setSelectedSensorPosition(0);
	}

	// ### Telemetry ###

	public void outputTelemetry() {
		SmartDashboard.putNumber("Manipulator/IntakePower", periodic.intakeMotorPower);
		SmartDashboard.putNumber("Manipulator/WristPower", periodic.wristMotorPower);
		SmartDashboard.putNumber("Manipulator/RawWristMotorPower", periodic.rawWristMotorPower);
		SmartDashboard.putNumber("Arm/encoder/wrist", periodic.wristEncoder);
		SmartDashboard.putNumber("Manipulator/IntakeMotorCurrent", periodic.intakeMotorCurrent);
		SmartDashboard.putNumber("Arm/error/wrist error", periodic.wristEncoderError);
		SmartDashboard.putNumber("Arm/encoder/wrist-D", periodic.desiredWristEncoder);
		SmartDashboard.putNumber("Arm/encoder/wrist-D", periodic.wristOffset);
	}


	public LogData getLogger() {
		return periodic;
	}
}
