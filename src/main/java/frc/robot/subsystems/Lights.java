package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;

public class Lights extends Subsystem {
    private static Lights instance = new Lights();
    public static Lights getInstance() { return instance; }

    public AddressableLED ledString;
    public AddressableLEDBuffer ledBuffer;
    public AddressableLED ledStringTwo;
    public AddressableLEDBuffer ledBufferTwo;
    public State state;
    public int rain;
    public double wantObject;

    public enum State {
        LIGHTS_RAINBOW,
        LIGHTS_WHITE,
        LIGHTS_YELLOW,
        LIGHTS_PURPLE,
        LIGHTS_GREEN
    }

    private Lights() {
        ledString = new AddressableLED(Constants.LIGHTS_ID);
        ledBuffer = new AddressableLEDBuffer(Constants.LIGHTS_LED_COUNT);
        ledString.setLength(ledBuffer.getLength());
        state = State.LIGHTS_RAINBOW;
        wantObject = 0.0;
        reset();

        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, 255, 255, 255);
        }

        ledString.setData(ledBuffer);

        ledString.start();
    }
    
    public void readPeriodicInputs() {}

    public void writePeriodicOutputs() {}

    public void outputTelemetry() {}

    public void setBothLedBuffersHSV(int i, int h, int s, int v) {
        ledBuffer.setHSV(i, h, s, v);
       // ledBufferTwo.setHSV(i, h, s, v);
    }

    public void setBothLedBuffersColor(int i, Color color) {
        ledBuffer.setLED(i, color);
       // ledBufferTwo.setLED(i, color);
    }

    public void outputTelemetry() {
        rain++;
        rain = rain % 180;
        if(DriverStation.isDisabled()){
            state = State.LIGHTS_WHITE;
        } else {
            if(Manipulator.getInstance().getTimeOfFlightActivated()){
                state = State.LIGHTS_GREEN;
            } else if(wantObject > 0.9){
                state = State.LIGHTS_YELLOW;
            } else if(wantObject < -0.9){
                state = State.LIGHTS_PURPLE;
            } else {
                state = State.LIGHTS_RAINBOW;
            }
        }
        switch(state) {
            case LIGHTS_RAINBOW:
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                setBothLedBuffersHSV(i, rain, 255, 230);
            }
                break;
            case LIGHTS_WHITE:
                for (int i = 0; i < ledBuffer.getLength(); i++) {
                    setBothLedBuffersColor(i, Color.kWhite);
                }
                break;
            case LIGHTS_PURPLE:
                for (int i = 0; i < ledBuffer.getLength(); i++) {
                    setBothLedBuffersColor(i, Color.kPurple);
                }
                break;
            case LIGHTS_YELLOW:
                for (int i = 0; i < ledBuffer.getLength(); i++) {
                    setBothLedBuffersColor(i, Color.kYellow);
                }
                break;
            case LIGHTS_GREEN:
                for (int i = 0; i < ledBuffer.getLength(); i++) {
                    setBothLedBuffersColor(i, Color.kGreen);
                }
                break;
        }
        ledString.setData(ledBuffer);
       // ledStringTwo.setData(ledBufferTwo);
    }

    public void reset() {
        ledBuffer = new AddressableLEDBuffer(Constants.LIGHTS_LED_COUNT);
        //ledBufferTwo = new AddressableLEDBuffer(Constants.LIGHTS_LED_COUNT_2);
        ledString.setLength(ledBuffer.getLength());
        //ledStringTwo.setLength(ledBufferTwo.getLength());
        ledString.setData(ledBuffer);
        //ledStringTwo.setData(ledBufferTwo);
        ledString.start();
        //ledStringTwo.start();
    }
}
