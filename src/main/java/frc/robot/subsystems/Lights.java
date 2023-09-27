package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;

public class Lights extends Subsystem {
    private static Lights instance = new Lights();

    public static Lights getInstance(){
        return instance;
    }

    public enum State {
        INIT,
        AUTO,
        TELEOP,
        CUBE,
        CONE,
        HAS_GAMEPIECE
    }

    private final AddressableLED leds;
    private final AddressableLEDBuffer buffer;
    private State state = State.INIT;
    private State previousState = State.INIT;
    private Timer timer = new Timer();

    public Lights() {
        leds = new AddressableLED(Constants.Lights.LIGHTS_ID);
        buffer = new AddressableLEDBuffer(Constants.Lights.NUM_LEDS);
        leds.setLength(Constants.Lights.NUM_LEDS);
        leds.start();
    }

    @Override
    public void readPeriodicInputs() {}

    @Override
    public void writePeriodicOutputs() {}

    @Override
    public void outputTelemetry() {
        switch (state) {
            case INIT:
                if(DriverStation.getAlliance() == Alliance.Blue) {
                    wave(100, Color.kBlack, Color.kBlue, 25.0, 2.0, 0.4);
                } else if (DriverStation.getAlliance() == Alliance.Red) {
                    wave(100, Color.kBlack, Color.kRed, 25.0, 2.0, 0.4);
                } else if (DriverStation.getAlliance() == Alliance.Invalid) {
                    wave(100, Color.kBlue, Color.kRed, 25.0, 2.0, 0.4);
                }
            break;
            case AUTO:
                if(DriverStation.getAlliance() == Alliance.Blue) {
                    breath(100, Color.kBlue, Color.kBlack, 1.0, Timer.getFPGATimestamp());
                } else {
                    breath(100, Color.kRed, Color.kBlack, 1.0, Timer.getFPGATimestamp());
                }
            break;
            case TELEOP:
                rainbow(100, 50.0, 1.5);
            break;
            case CONE:
                if (timer.get() <= 5.0) {
                    strobe(100, Color.kYellow, 0.5);
                } else {
                    timer.stop();
                    setState(previousState);
                }
            break;
            case CUBE:
                if (timer.get() <= 5.0) {
                    strobe(100, Color.kPurple, 0.5);
                } else {
                    timer.stop();
                    setState(previousState);
                }
            break;
            case HAS_GAMEPIECE:
                wave(100, Color.kGreen, Color.kPurple, 25.0, 2.0, 0.4);
            break;
        }
        leds.setData(buffer);
    }

    private void solid(double percent, Color color) {
        for (int i = 0; i<MathUtil.clamp(Constants.Lights.NUM_LEDS, percent, Constants.Lights.NUM_LEDS); i++) {
            buffer.setLED(i, color);
        }
    }

    private void strobe(double percent, Color color, double duration) {
        boolean on = ((Timer.getFPGATimestamp() % duration) / duration) > 0.5;
        solid(percent, on ? color: Color.kBlack);
    }

    private void breath(double percent, Color c1, Color c2, double duration, double timestamp) {
        double x = ((timestamp % duration) / duration) * 2.0 * Math.PI;
        double ratio = (Math.sin(x) + 1.0) / 2.0;
        double red = (c1.red * (1-ratio)) + (c2.red * ratio);
        double green = (c1.green * (1-ratio)) + (c2.green * ratio);
        double blue = (c1.blue * (1-ratio)) + (c2.blue * ratio);
        solid(percent, new Color(red, green, blue));
    }

    private void rainbow(double percent, double cycleLength, double duration) {
        double x = (1-((Timer.getFPGATimestamp() / duration) % 1.0)) * 180.0;
        double xDiffPerLed = 180.0 / cycleLength;
        for (int i = 0; i<MathUtil.clamp(Constants.Lights.NUM_LEDS, percent, Constants.Lights.NUM_LEDS); i++) {
            x += xDiffPerLed;
            x %= 180.0;
            if (i >= 0) {
                buffer.setHSV(i, (int)x, 255, 255);
            }
        }
    }

    private void wave(double percent, Color c1, Color c2, double cycleLength, double duration, double waveExponent) {
        double x = (1 - ((Timer.getFPGATimestamp() % duration) / duration)) * 2.0 * Math.PI;
        double xDiffPerLed = (2.0 * Math.PI) / cycleLength;
        for (int i = 0; i<MathUtil.clamp(Constants.Lights.NUM_LEDS, percent, Constants.Lights.NUM_LEDS); i++) {
          x += xDiffPerLed;
          if (i >= 0) {
            double ratio = (Math.pow(Math.sin(x), waveExponent) + 1.0) / 2.0;
            if (Double.isNaN(ratio)) {
              ratio = (-Math.pow(Math.sin(x + Math.PI), waveExponent) + 1.0) / 2.0;
            }
            if (Double.isNaN(ratio)) {
              ratio = 0.5;
            }
            double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
            double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
            double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
            buffer.setLED(i, new Color(red, green, blue));
          }
        }
      }

    public void setState(State state) {
        this.state = state;
    }

    public State getState() {
        return state;
    }

    public void setCube() {
        if (state != State.CONE && state != State.CUBE) {
            previousState = state;
        }
        timer.reset();
        timer.start();
        setState(State.CUBE);
    }

    public void setCone() {
        if (state != State.CONE && state != State.CUBE) {
            previousState = state;
        }
        timer.reset();
        timer.start();
        setState(State.CONE);
    }

    public void setGamepiece(boolean hasGamePiece) {
        if(state != State.HAS_GAMEPIECE) {
            previousState = state;
        }
        if (hasGamePiece) {
            setState(State.HAS_GAMEPIECE);
        } else {
            setState(previousState);
        }
    }

    @Override
    public void reset() {}
    
}
