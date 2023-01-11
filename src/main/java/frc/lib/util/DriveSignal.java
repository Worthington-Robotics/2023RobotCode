package frc.lib.util;

/**
 * A drivetrain command consisting of the linear and angular motor settings and whether the brake mode is enabled.
 */
public class DriveSignal {
    protected double mRightMotor;
    protected double mLeftMotor;
    protected boolean mBrakeMode;

    public DriveSignal(double linear, double angular) {
        this(linear, angular, false);
    }

    public DriveSignal(double left, double right, boolean brakeMode) {
        mLeftMotor = left;
        mRightMotor = right;
        mBrakeMode = brakeMode;
    }

    public static final DriveSignal NEUTRAL = new DriveSignal(0, 0);
    public static final DriveSignal BRAKE = new DriveSignal(0, 0, true);

    //getters
    public double getLeft() {
        return mLeftMotor;
    }

    public double getRight() {
        return mRightMotor;
    }

    public boolean getBrakeMode() {
        return mBrakeMode;
    }

    //manipulation functions
    public DriveSignal invert() {
        mLeftMotor *= -1;
        mRightMotor *= -1;
        return this;
    }

    @Override
    public String toString() {
        return "L: " + mLeftMotor + ", R: " + mRightMotor + (mBrakeMode ? ", BRAKE" : "");
    }
}