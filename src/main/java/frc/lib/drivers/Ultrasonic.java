package frc.lib.drivers;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalOutput;

public class Ultrasonic {

    private DigitalOutput outputPin;
    private Counter inputPin;
    private double distance;

    public Ultrasonic(int inputPin, int outputPin) {
        this.outputPin = new DigitalOutput(outputPin);
        this.inputPin = new Counter(inputPin);
        this.inputPin.setSemiPeriodMode(true);
    }

    public void update() {
        outputPin.set(true);
        distance = inputPin.getPeriod() * 1000000 / 148;
        outputPin.set(false);
    }

    public double getDistance() {
        update();
        return distance;
    }

}
