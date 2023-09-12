package frc.robot.subsystems.arm;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import frc.lib.loops.ILooper;
import frc.lib.loops.Loop;
import frc.robot.subsystems.Subsystem;

public class ArmNew extends Subsystem {
    private static ArmNew instance = new ArmNew();
    public static ArmNew getInstance() {return instance;}

	private ArmKinematics kinematics = new ArmKinematics();
	Vector<N3> angles = kinematics.inverseSafe(ArmPoseNew.Preset.MID_CONE.getPose2d());
	private ArmVisualizer visualizer = new ArmVisualizer(angles);

    @Override
    public void readPeriodicInputs() {
        // TODO Auto-generated method stub
        
    }

    public void registerEnabledLoops(ILooper enalbedLooper) {
        enalbedLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                // TODO Auto-generated method stub
                
            }
            @Override
            public void onLoop(double timestamp) {
                double x = Math.sin(timestamp);
                visualizer.update(kinematics.inverseSafe(new Pose2d(0.9+(0.3*x), 0.5 *x, new Rotation2d(0))));
            }
            @Override
            public void onStop(double timestamp) {
                // TODO Auto-generated method stub
                
            }
        });
    }

    @Override
    public void writePeriodicOutputs() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void outputTelemetry() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void reset() {
        // TODO Auto-generated method stub
        
    }
    
}
