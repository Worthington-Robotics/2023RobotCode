package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.loops.ILooper;
import frc.lib.loops.Loop;
import frc.robot.Constants;

public class Lights extends Subsystem {
    private static Lights instance = new Lights();
    public static Lights getInstance() { return instance; }

    public AddressableLED ledString;
    public AddressableLEDBuffer ledBuffer;
    public State state;
    public double h;

    public enum State {
        LIGHTS_RAINBOW,
        LIMELIGHT_TARGETING,
        LIGHTS_WHITE
    }

    private Lights() {
        ledString = new AddressableLED(Constants.LIGHTS_ID);
        ledBuffer = new AddressableLEDBuffer(Constants.LIGHTS_LED_COUNT);
        ledString.setLength(ledBuffer.getLength());
        state = State.LIGHTS_RAINBOW;
        reset();
    }

    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                state = State.LIGHTS_RAINBOW;
            }
            @Override
            public void onLoop(double timestamp) {
                
            }
            @Override
            public void onStop(double timestamp) {

            }
        });
    }
    

    public void readPeriodicInputs() {}

    public void writePeriodicOutputs() {
    }

    public void outputTelemetry() {
        double timestamp = Timer.getFPGATimestamp();
        if(DriverStation.isDisabled()){
            state = State.LIGHTS_WHITE;
        }
        switch(state) {
            case LIGHTS_RAINBOW:
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                double speed = 10.0;
                double length = 0.1;
                double h = Math.abs(((timestamp - (length*i)) % speed) / speed);
                ledBuffer.setHSV(i, (int)(h * 180.0), 255, 230);
            }
                break;
            case LIGHTS_WHITE:
                for (int i = 0; i < ledBuffer.getLength(); i++) {
                    ledBuffer.setHSV(i, 0, 0, 255);
                }
                break;
            case LIMELIGHT_TARGETING:
                for (int i = 0; i < ledBuffer.getLength(); i++) {
                    if (VisionLink.getInstance().hasTarget()) {
                        final double error = Arm.getInstance().getTurretError();
                        if (error <= Constants.ANGLE_ACCEPTANCE) {
                            ledBuffer.setRGB(i, 174, 255, 54);
                        } else {
                            ledBuffer.setRGB(i, 255, 200, 0);
                        }
                    } else {
                        ledBuffer.setRGB(i, 250, 78, 35);
                    }
                }
                break;
        }
        ledString.setData(ledBuffer);

    }

    public void setLimelightTargetColor() {
        state = State.LIMELIGHT_TARGETING;
    }

    public void setRainbowLights() {
        state = State.LIGHTS_RAINBOW;
    }

    public void reset() {
        ledBuffer = new AddressableLEDBuffer(Constants.LIGHTS_LED_COUNT);
        ledString.setLength(ledBuffer.getLength());
        ledString.setData(ledBuffer);
        ledString.start();
    }
}