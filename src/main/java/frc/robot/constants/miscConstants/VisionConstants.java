package frc.robot.constants.miscConstants;
/**
 * Container class for vision constants.
 */

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Robot.RobotId;

public class VisionConstants {
  public static final Pose3d m_tagPose = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)); 
  //TODO: need to figure out how to represent the transform from the camera to the robot
  public static final Transform3d m_cameraToRobot = new Transform3d(); 

  public static void update(RobotId robotId) {
    
  }
 
}