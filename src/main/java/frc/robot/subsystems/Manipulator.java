package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.Arm.ArmMode;

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
		public ManipulatorMode currentMode = ManipulatorMode.OPEN_LOOP;
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
		intakeTOF = new TimeOfFlight(1);
		wristMotor = new TalonFX(Constants.WRIST_MOTOR_ID, "Default Name");
		wristMotor.setNeutralMode(NeutralMode.Brake);

		intakeMotor = new TalonFX(Constants.INTAKE_MOTOR_ID, "Default Name");
		intakeMotor.setNeutralMode(NeutralMode.Brake);
	}

	public void readPeriodicInputs() {
		periodic.wristEncoder = wristMotor.getSelectedSensorPosition();
		periodic.rawWristMotorPower = HIDHelper.getAxisMapped(Constants.MASTER.getRawAxis(3), 1,0);
	}

	public void writePeriodicOutputs() {
		intakeMotor.set(ControlMode.PercentOutput, periodic.intakeMotorPower);
		if (Arm.getInstance().getMode().ordinal() < ArmMode.CLOSED_LOOP.ordinal() ) {
			wristMotor.set(ControlMode.PercentOutput, periodic.wristMotorPower);
		} else {
			wristMotor.set(ControlMode.Position, periodic.desiredWristEncoder);
		}
	}

	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(new Loop() {
			@Override
			public void onStart(double timestamp) {
				reset();
				wristAnglePID();
			}

			@Override
			public void onLoop(double timestamp) {
				switch (periodic.currentMode) {
					case OPEN_LOOP:
						break;
					case OPEN_CLOSED_LOOP:
						periodic.desiredWristEncoder = convertRawWristPowerIntoEncoder(periodic.rawWristMotorPower);
						break;
					case CLOSED_LOOP:
						periodic.desiredWristEncoder = Arm.ArmPoses[Arm.getInstance().getPose().ordinal()][2];
						break;
				}
			}

			@Override
			public void onStop(double timestamp) {
				reset();
				wristAnglePID();
			}
		});
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

	// Set the desired angle of the wrist
	public void setDesiredWristAngle(double thetaEncoder) {
		periodic.desiredWristDegree = thetaEncoder / Constants.WRIST_ENCODER_PER_DEGREE;
		periodic.desiredWristEncoder = thetaEncoder;
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
		wristMotor.config_kI(0,0);
		wristMotor.config_kD(0,0);
		wristMotor.config_kF(0,0);
	}


	public void reset() { 
		periodic.currentMode = ManipulatorMode.OPEN_CLOSED_LOOP;
	}

	public void resetManipulatorEncoder(){
		wristMotor.setSelectedSensorPosition(0);
	}

	// ### Telemetry ###

	public void outputTelemetry() {
		SmartDashboard.putNumber("Manipulator/IntakePower", periodic.intakeMotorPower);
		SmartDashboard.putNumber("Manipulator/WristPower", periodic.wristMotorPower);
		SmartDashboard.putNumber("Manipulator/RawWristMotorPower", periodic.rawWristMotorPower);
		SmartDashboard.putNumber("Manipulator/DesiredWristEncoder", periodic.desiredWristEncoder);
		SmartDashboard.putNumber("Manipulator/WristEncoder", periodic.wristEncoder);
		SmartDashboard.putNumber("Manipulator/TOFDistance", intakeTOF.getRange());
	}

	public LogData getLogger() {
		return periodic;
	}
}
