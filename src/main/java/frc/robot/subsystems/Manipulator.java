package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

public class Manipulator extends Subsystem {
	private static Manipulator instance = new Manipulator();
	public static Manipulator getInstance() { return instance; }

	private TimeOfFlight tof;
	private TalonFX intakeWheelMotor;
	private TalonFX intakeWristMotor;

	public class SuperIO extends Subsystem.PeriodicIO {
		double tofRange = 0;
		double intakePower = 0;
        double wristPower = 0;
	}

	private SuperIO periodic;

	public Manipulator() {
		periodic = new SuperIO();
		intakeWheelMotor = new TalonFX(Constants.INTAKE_WHEEL_ID);
		tof = new TimeOfFlight(Constants.MANIPULATOR_TOF_ID);
		tof.setRangingMode(RangingMode.Short, 40);
	}

	public void readPeriodicInputs() {
	    periodic.tofRange = tof.getRange();
	}

	public void writePeriodicOutputs() {
		intakeWheelMotor.set(ControlMode.PercentOutput, periodic.intakePower);
        intakeWristMotor.set(ControlMode.PercentOutput, periodic.wristPower);
	}


	public void setIntakePower(double power) {
		periodic.intakePower = power;
	}

    public void increasePivotPower(){
        periodic.wristPower += Constants.PIVOT_INCREMENT;
        if(periodic.wristPower >= Constants.MAX_PIVOT_POWER){
            periodic.wristPower = Constants.MAX_PIVOT_POWER;
        }
    }

    public void decreasePivotPower(){
        periodic.wristPower -= Constants.PIVOT_INCREMENT;
        if(periodic.wristPower <= Constants.MIN_PIVOT_POWER){
            periodic.wristPower = Constants.MIN_PIVOT_POWER;
        }
    }


	// Returns if the game piece has completed its intake cycle
	public boolean isFinished() {
		return false;
	}

	public void reset() {
		periodic = new SuperIO();
	}

	// ### Telemetry ###

	public void outputTelemetry() {
		SmartDashboard.putNumber("Manipulator/IntakePower", periodic.intakePower);
        SmartDashboard.putNumber("Manipulator/WristPower", periodic.wristPower);
	}

	public LogData getLogger() {
		return periodic;
	}
}
