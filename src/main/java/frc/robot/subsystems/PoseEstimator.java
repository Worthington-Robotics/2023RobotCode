package frc.robot.subsystems;

import frc.lib.geometry.Twist2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Rotation2d;
import frc.lib.loops.ILooper;
import frc.lib.loops.Loop;
import frc.lib.math.*;
import frc.robot.Kinematics;

import java.util.Map;

public class PoseEstimator extends Subsystem {
    private static PoseEstimator instance = new PoseEstimator();
    public static PoseEstimator getInstance() { return instance; }

    private static final int OBSERVATION_BUFFER_SIZE = 10;

    private InterpolatingTreeMap<InterpolatingDouble, Pose2d> field_to_vehicle;
    private double left_encoder_prev_distance = 0.0;
    private double right_encoder_prev_distance = 0.0;
    private double heading_zero = 0.0;
    private PoseIO periodic;

    @Override
    public void registerEnabledLoops(ILooper looper) {
        looper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                periodic.distance_driven = 0.0;
                left_encoder_prev_distance = DriveTrain.getInstance().getLeftEncoderDistance();
                right_encoder_prev_distance = DriveTrain.getInstance().getRightEncoderDistance();
            }
    
            @Override
            public void onLoop(double timestamp) {
                synchronized (this) {
                    final double gyro_angle = DriveTrain.getInstance().getRawHeading();
                    final double left_distance = DriveTrain.getInstance().getLeftEncoderDistance();
                    final double right_distance = DriveTrain.getInstance().getRightEncoderDistance();
                    final double delta_left = left_distance - left_encoder_prev_distance;
                    final double delta_right = right_distance - right_encoder_prev_distance;
                    final double delta_heading = gyro_angle - heading_zero;
                    final Twist2d odometry_velocity = generateOdometryFromSensors(delta_left, delta_right,
                        Rotation2d.fromDegrees(delta_heading));
                    addObservations(timestamp, Kinematics.integrateForwardKinematics(getLatestFieldToVehicle().getValue(), odometry_velocity));
                    left_encoder_prev_distance = left_distance;
                    right_encoder_prev_distance = right_distance;
                    periodic.odometry = getLatestFieldToVehicle().getValue();
                    outputTelemetry();
                }
            }
    
            @Override
            public void onStop(double timestamp) {}
        });
    }

    private PoseEstimator() {
        reset(0, Pose2d.identity());
        periodic = new PoseIO();
    }

    public synchronized void reset(double start_time, Pose2d initial_field_to_vehicle) {
        periodic = new PoseIO();
        heading_zero = DriveTrain.getInstance().getRawHeading();
        field_to_vehicle = new InterpolatingTreeMap<>(OBSERVATION_BUFFER_SIZE);
        field_to_vehicle.put(new InterpolatingDouble(start_time), initial_field_to_vehicle);
        periodic.distance_driven = 0.0;
    }

    public synchronized Map.Entry<InterpolatingDouble, Pose2d> getLatestFieldToVehicle() {
        return field_to_vehicle.lastEntry();
    }

    public synchronized Pose2d getFieldToVehicle(double timestamp) {
        return field_to_vehicle.getInterpolated(new InterpolatingDouble(timestamp));
    }

    public synchronized Twist2d generateOdometryFromSensors(double left_encoder_delta_distance, double
            right_encoder_delta_distance, Rotation2d current_gyro_angle) {
        final Pose2d last_measurement = getLatestFieldToVehicle().getValue();
        final Twist2d delta = Kinematics.forwardKinematics(last_measurement.getRotation(),
                left_encoder_delta_distance, right_encoder_delta_distance,
                current_gyro_angle);
        periodic.distance_driven += delta.dx; //do we care about dy here?
        return delta;
    }

    public synchronized void addObservations(double timestamp, Pose2d observation) {
        field_to_vehicle.put(new InterpolatingDouble(timestamp), observation);
    }

    public double getDistanceDriven() {
        return periodic.distance_driven;
    }

    public double getAbsoluteHeading() {
        return (periodic.odometry.getRotation().getDegrees() + 360) % 360;
    }

    @Override
    public void outputTelemetry() {
        final double x = periodic.odometry.getTranslation().x();
        final double y = periodic.odometry.getTranslation().y();
        final double theta = (periodic.odometry.getRotation().getDegrees() + 360) % 360;
        SmartDashboard.putNumber("Drive/Pose/X", x);
        SmartDashboard.putNumber("Drive/Pose/Y", y);
        SmartDashboard.putNumber("Drive/Pose/Theta", theta);
        SmartDashboard.putNumberArray("Drive/Pose/Postion", new double[] {x / 1705, y / 1705, theta});
        SmartDashboard.putNumber("Drive/Pose/ThetaZero", heading_zero);
    }

    @Override
    public void reset() {
        reset(Timer.getFPGATimestamp(), Pose2d.identity());
    }
  
    public class PoseIO extends Subsystem.PeriodicIO {
        public Pose2d odometry = Pose2d.identity();
        public double distance_driven = 0.0;
    }

    @Override
    public void readPeriodicInputs() {}

    @Override
    public void writePeriodicOutputs() {}

    public LogData getLogger() {
        return periodic;
    } 
}
