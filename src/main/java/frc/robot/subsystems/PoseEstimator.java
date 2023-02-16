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

    private static PoseEstimator m_instance = new PoseEstimator();

    private static final int observation_buffer_size_ = 10;

    private InterpolatingTreeMap<InterpolatingDouble, Pose2d> field_to_vehicle_;
    private double left_encoder_prev_distance_ = 0.0;
    private double right_encoder_prev_distance_ = 0.0;
    private PoseIO periodic;


    @Override
    public void registerEnabledLoops(ILooper looper) {
        looper.register(new Loop() {

            @Override
            public void onStart(double timestamp) {
                periodic.distance_driven = 0.0;
                left_encoder_prev_distance_ = DriveTrain.getInstance().getLeftEncoderDistance();
                right_encoder_prev_distance_ = DriveTrain.getInstance().getRightEncoderDistance();
            }
    
            @Override
            public void onLoop(double timestamp) {
                synchronized (this) {
                    final Rotation2d gyro_angle = DriveTrain.getInstance().getHeading();
                    final double left_distance = DriveTrain.getInstance().getLeftEncoderDistance();
                    final double right_distance = DriveTrain.getInstance().getRightEncoderDistance();
                    final double delta_left = left_distance - left_encoder_prev_distance_;
                    final double delta_right = right_distance - right_encoder_prev_distance_;
                    final Twist2d odometry_velocity = generateOdometryFromSensors(delta_left, delta_right, gyro_angle);
                    addObservations(timestamp, Kinematics.integrateForwardKinematics(getLatestFieldToVehicle().getValue(), odometry_velocity));
                    left_encoder_prev_distance_ = left_distance;
                    right_encoder_prev_distance_ = right_distance;
                    periodic.odometry = getLatestFieldToVehicle().getValue();
                    outputTelemetry();
                }
            }
    
            @Override
            public void onStop(double timestamp) {
    
            }
        });
    }
    

    public static PoseEstimator getInstance() {
        return m_instance;
    }

    private PoseEstimator() {
        reset(0, Pose2d.identity());
        periodic = new PoseIO();
    }

    public synchronized void reset(double start_time, Pose2d initial_field_to_vehicle) {
        periodic = new PoseIO();
        field_to_vehicle_ = new InterpolatingTreeMap<>(observation_buffer_size_);
        field_to_vehicle_.put(new InterpolatingDouble(start_time), initial_field_to_vehicle);
        periodic.distance_driven = 0.0;
    }

    public synchronized Map.Entry<InterpolatingDouble, Pose2d> getLatestFieldToVehicle() {
        return field_to_vehicle_.lastEntry();
    }

    public synchronized Pose2d getFieldToVehicle(double timestamp) {
        return field_to_vehicle_.getInterpolated(new InterpolatingDouble(timestamp));
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
        field_to_vehicle_.put(new InterpolatingDouble(timestamp), observation);
    }

    public double getDistanceDriven() {
        return periodic.distance_driven;
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
    public void readPeriodicInputs() {
        // TODO Auto-generated method stub

    }

    @Override
    public void writePeriodicOutputs() {
        // TODO Auto-generated method stub

    }

    public LogData getLogger() {
        return periodic;
    } 
}