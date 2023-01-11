package frc.lib.util;

import edu.wpi.first.wpilibj.Joystick;

public class HIDHelper {

    /**
     * @param constants HID constants
     * @return double array following X(0), Y(1), Z(2)
     */
    public static double[] getAdjStick(HIDConstants constants) {
        double[] out = new double[4];
        out[0] = applyScalarCut(constants.joystick.getX(), constants.deadBand, constants.scalarCutX, constants.polyFunct);
        out[1] = applyScalarCut(constants.joystick.getY(), constants.deadBand, constants.scalarCutY, constants.polyFunct);
        out[2] = applyScalarCut(constants.joystick.getZ(), constants.deadBand, constants.scalarCutZ, constants.polyFunct);
        out[3] = applyScalarCut(constants.joystick.getThrottle(), constants.deadBand, constants.scalarCutGas, constants.polyFunct);
        return out;
    }

    private static double applyScalarCut(double stickInput, double deadBand, double scalarCut, int pow) {
        return evalDeadBand(stickInput, deadBand, pow) * scalarCut;
    }

    // figures out if the stick value is within the deadband
    static double evalDeadBand(double stickInpt, double deadBand, int pow) {
        if (Math.abs(stickInpt) < deadBand) {
            return 0;
        } else {
            if (stickInpt < 0) {
                return -Math.abs(Math.pow(stickInpt, pow));
            } else {
                return Math.abs(Math.pow(stickInpt, pow));
            }
        }
    }

    public static double getAxisMapped(double input, double min_output, double max_output) {
        return (input + 1) * (max_output - min_output) / (2) + min_output;
    }


    public static class HIDConstants {
        private Joystick joystick;
        private double deadBand, scalarCutX, scalarCutY, scalarCutZ, scalarCutGas;
        private int polyFunct;

        public HIDConstants(Joystick joystick, double deadBand, double scalarCutX, double scalarCutY, double scalarCutZ, int polyFunct) {
            this.joystick = joystick;
            this.deadBand = deadBand;
            this.scalarCutX = scalarCutX;
            this.scalarCutY = scalarCutY;
            this.scalarCutZ = scalarCutZ;
            this.polyFunct = polyFunct;
        }

        public HIDConstants(Joystick joystick, double deadBand, double scalarCutX, double scalarCutY, double scalarCutZ, double scalarCutGas, int polyFunct) {
            this.joystick = joystick;
            this.deadBand = deadBand;
            this.scalarCutX = scalarCutX;
            this.scalarCutY = scalarCutY;
            this.scalarCutZ = scalarCutZ;
            this.scalarCutGas = scalarCutGas;
            this.polyFunct = polyFunct;
        }

        public static Joystick getJoystick(HIDConstants hid) {
            return hid.joystick;
        }
    }

}
