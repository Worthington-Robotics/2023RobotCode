package frc.lib.drivers;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimDevice.Direction;
 
public class SimTimeOfFlight {
    private TimeOfFlight tof;
    private SimDevice sim;
    private SimDouble simDist;

    public SimTimeOfFlight(int sensorId) {
        sim = SimDevice.create("TOF sensor", sensorId);
        if (sim == null) {
            tof = new TimeOfFlight(sensorId);
        } else {
            // FIXME: Function signature
            simDist = sim.createDouble("Distance", Direction.kInput, 0.0);
        }
    }

    public double getRange() {
        if (sim == null) {
            return tof.getRange();
        } else {
            return simDist.get();
        }
    }

    public void setRangingMode(RangingMode mode, double sampleTime) {
        if (sim == null) {
            tof.setRangingMode(mode, sampleTime);
        }
    } 
}
