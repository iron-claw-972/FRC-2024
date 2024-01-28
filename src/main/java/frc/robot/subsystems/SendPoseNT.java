package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SendPoseNT extends SubsystemBase {
    NetworkTableInstance m_NetworkTableInstance;
    
    DoublePublisher xPub;
    DoublePublisher yPub;
    DoublePublisher rotationPub;

    Drivetrain m_Drivetrain;


    public SendPoseNT(Drivetrain drivetrain) {
        m_NetworkTableInstance = NetworkTableInstance.getDefault();

        NetworkTable table = m_NetworkTableInstance.getTable("robot_pose2D");

        xPub = table.getDoubleTopic("x").publish();
        yPub = table.getDoubleTopic("y").publish();
        rotationPub = table.getDoubleTopic("rotation").publish();


    }
    
    @Override
    public void periodic() {
        Pose2d robotPose2d = m_Drivetrain.getPose();
        
        // Update stuff here
        xPub.set(robotPose2d.getX());
        yPub.set(robotPose2d.getY());
        rotationPub.set(robotPose2d.getRotation().getDegrees());


    }
}
