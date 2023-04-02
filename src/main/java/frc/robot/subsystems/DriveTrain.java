package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.math.controller.LinearPlantInversionFeedforward;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.lib.loops.ILooper;
import frc.lib.loops.Loop;
import frc.lib.util.HIDHelper;
import frc.robot.Constants;

public class DriveTrain extends Subsystem {
    private static DriveTrain instance = new DriveTrain();
    public static DriveTrain getInstance() { return instance; }
    private DriveTrainIO periodic = new DriveTrainIO();

    public enum State {
    }

    public class DriveTrainIO {
        public State state;
    }

    private DriveTrain() {

        reset();
    }

    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
            }
            @Override
            public void onLoop(double timestamp) {
            }
            @Override
            public void onStop(double timestamp) {

            }
        });
    }
    

    public void readPeriodicInputs() {
    }

    public void writePeriodicOutputs() {
    }

    public void outputTelemetry() {
    }

    public void reset() {
    }
}