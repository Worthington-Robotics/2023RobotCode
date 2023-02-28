package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
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
	private TimeOfFlight intakeTOF;

	public enum ManipulatorMode {
		OPEN_LOOP,
		OPEN_CLOSED_LOOP,
		CLOSED_LOOP
    }

	public class ManipulatorIO extends PeriodicIO {
		public ManipulatorMode currentMode = ManipulatorMode.OPEN_CLOSED_LOOP;
		double wristMotorPower;
		double intakeMotorPower;
		double rawWristMotorPower;
		double desiredWristEncoder;
		double desiredWristDegree;
		double wristEncoderError;
		double wristEncoder;
	}

	public Manipulator() {
		periodic = new ManipulatorIO();

		wristMotor = new TalonFX(Constants.WRIST_MOTOR_ID, "Default Name");
		wristMotor.setNeutralMode(NeutralMode.Brake);

		intakeMotor = new TalonFX(Constants.INTAKE_MOTOR_ID, "Default Name");
		intakeMotor.setNeutralMode(NeutralMode.Brake);

		intakeTOF = new TimeOfFlight(Constants.BACKSTOP_TOF_ID);
	}

	public void readPeriodicInputs() {
		periodic.rawWristMotorPower = HIDHelper.getAxisMapped(Constants.MASTER.getRawAxis(3), 1,0);
	}

	public void writePeriodicOutputs() {
		intakeMotor.set(ControlMode.PercentOutput, periodic.intakeMotorPower);
		wristMotor.set(ControlMode.Position, periodic.desiredWristEncoder);
		if (periodic.currentMode == ManipulatorMode.CLOSED_LOOP) {
			wristMotor.set(ControlMode.PercentOutput, periodic.wristMotorPower);
		}
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
						setWristPower(periodic.rawWristMotorPower);
						break;
					case OPEN_CLOSED_LOOP:
						periodic.desiredWristEncoder = convertRawWristPowerIntoEncoder(periodic.rawWristMotorPower);
						wristAnglePID();
						break;
					case CLOSED_LOOP:
						wristAnglePID();
						break;
				}
			}

			@Override
			public void onStop(double timestamp) {
				reset();
			}
		});
	}

	// Convert joystick values into motor powers
	public double convertRawWristPowerIntoEncoder(double inputPower) {
		return inputPower * 1000.0;
	}

	// Set the intake demand to the specified value
	public void setIntakePower(double power) {
		if (intakeTOF.getRange() <= Constants.INTAKE_ACCEPTANCE_RANGE) {
			periodic.intakeMotorPower = power;
		} else {
			periodic.intakeMotorPower = 0.0;
		}
	}

	public void setWristPower(double power){
		periodic.wristMotorPower = power;
	}

	// Set the desired angle of the wrist
	public void setDesiredWristAngle(double theta) {
		periodic.desiredWristDegree = theta;
		periodic.desiredWristEncoder = theta * Constants.WRIST_ENCODER_PER_DEGREE;
		periodic.wristEncoderError = periodic.desiredWristEncoder - periodic.wristEncoder;
	}

	public void setOpenLoop() {
		periodic.currentMode = ManipulatorMode.OPEN_LOOP;
	}

	public void setOpenClosedLoop() {
		periodic.currentMode = ManipulatorMode.OPEN_CLOSED_LOOP;
	}

	public void setClosedLoop() {
		periodic.currentMode = ManipulatorMode.CLOSED_LOOP;
	}

	// Getters

	public double getWristEncoderError(){
		return periodic.wristEncoderError;
	}

	// PID
	
	public void wristAnglePID() {
		wristMotor.config_kP(0, Constants.WRIST_PIVOT_KP);
	}


	public void reset() { 
		periodic.currentMode = ManipulatorMode.OPEN_CLOSED_LOOP;
	}

	// ### Telemetry ###

	public void outputTelemetry() {
		SmartDashboard.putNumber("Manipulator/IntakePower", periodic.intakeMotorPower);
		SmartDashboard.putNumber("Manipulator/WristPower", periodic.wristMotorPower);
		SmartDashboard.putNumber("Manipulator/RawWristMotorPower", periodic.rawWristMotorPower);
		SmartDashboard.putNumber("Manipulator/DesiredWristEncoder", periodic.desiredWristEncoder);
	}

	public LogData getLogger() {
		return periodic;
	}
}
