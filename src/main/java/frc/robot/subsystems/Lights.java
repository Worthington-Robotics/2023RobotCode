package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.lib.loops.ILooper;
import frc.lib.loops.Loop;
import frc.lib.util.HIDHelper;
import frc.robot.Constants;

public class Lights extends Subsystem {
    private static Lights instance = new Lights();
    public static Lights getInstance() { return instance; }

    public AddressableLED ledString;
    public AddressableLEDBuffer ledBuffer;
    public State state;
    public int rain;
    public double wantObject;

    public enum State {
        LIGHTS_RAINBOW,
        LIGHTS_WHITE,
        LIGHTS_YELLOW,
        LIGHTS_PURPLE
    }

    private Lights() {
        ledString = new AddressableLED(Constants.LIGHTS_ID);
        ledBuffer = new AddressableLEDBuffer(Constants.LIGHTS_LED_COUNT);
        ledString.setLength(ledBuffer.getLength());
        state = State.LIGHTS_RAINBOW;
        wantObject = 0.0;
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
        wantObject = HIDHelper.getAxisMapped(Constants.MASTER.getRawAxis(3), -1, 1);
    }

    public void writePeriodicOutputs() {
    }

    public void outputTelemetry() {
        rain++;
        rain = rain % 180;
        if(DriverStation.isDisabled()){
            state = State.LIGHTS_WHITE;
        } else {
            if(wantObject > 0.9){
                state = State.LIGHTS_PURPLE;
            } else if(wantObject < -0.9){
                state = State.LIGHTS_YELLOW;
            } else {
                state = State.LIGHTS_RAINBOW;
            }
        }
        switch(state) {
            case LIGHTS_RAINBOW:
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                ledBuffer.setHSV(i, rain, 255, 230);
            }
                break;
            case LIGHTS_WHITE:
                for (int i = 0; i < ledBuffer.getLength(); i++) {
                    ledBuffer.setLED(i, Color.kWhite);
                }
                break;
            case LIGHTS_PURPLE:
                for (int i = 0; i < ledBuffer.getLength(); i++) {
                    ledBuffer.setLED(i, Color.kPurple);
                }
                break;
            case LIGHTS_YELLOW:
                for (int i = 0; i < ledBuffer.getLength(); i++) {
                    ledBuffer.setLED(i, Color.kYellow);
                }
                break;
        }
        ledString.setData(ledBuffer);

    }

    public void reset() {
        ledBuffer = new AddressableLEDBuffer(Constants.LIGHTS_LED_COUNT);
        ledString.setLength(ledBuffer.getLength());
        ledString.setData(ledBuffer);
        ledString.start();
    }

    

}