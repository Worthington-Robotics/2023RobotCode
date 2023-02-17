package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class Lights extends Subsystem {
    private static Lights instance = new Lights();
    public static Lights getInstance() { return instance; }

    public AddressableLED ledString;
    public AddressableLEDBuffer ledBuffer;

    private Lights() {
        ledString = new AddressableLED(0);
        ledBuffer = new AddressableLEDBuffer(60);

        ledString.setLength(ledBuffer.getLength());

        for (var i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, 0, 0, 255);
        }

        ledString.setData(ledBuffer);

        ledString.start();
    }
    
    public void readPeriodicInputs() {}

    public void writePeriodicOutputs() {}

    public void outputTelemetry() {}

    public void reset() {}
}
