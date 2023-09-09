package frc.robot.commands.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.Vision;

public class ReturnData extends CommandBase{
  private final Vision m_vision; 

  public ReturnData(Vision vision){
    m_vision = vision;
  }
  
  @Override
  public void execute() {
    //put the offsets and area on SmartDashboard for testing 
    SmartDashboard.putNumber("X offset degrees", m_vision.getHorizontalOffsetDegrees()); 
    SmartDashboard.putNumber("Y offset degrees", m_vision.getVerticalOffsetDegrees()); 
    SmartDashboard.putNumber("Area", m_vision.getTargetAreaPercentage());
    SmartDashboard.putNumberArray("Robot pose", m_vision.getRobotPose()); 
 
    // Print data
    System.out.println("X offset degrees: " + m_vision.getHorizontalOffsetDegrees()); 
    System.out.println("Y offset degrees: " + m_vision.getVerticalOffsetDegrees()); 
    System.out.println("Area: " + m_vision.getTargetAreaPercentage());
    System.out.println("Robot pose: " + m_vision.getRobotPose()); 
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return false; 
  }


}

