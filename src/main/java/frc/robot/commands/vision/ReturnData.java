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
    // Store pose to make it easier to use
    double[] pose = m_vision.getRobotPose();

    //put the offsets and area on SmartDashboard for testing 
    SmartDashboard.putNumber("X offset degrees", m_vision.getHorizontalOffsetDegrees()); 
    SmartDashboard.putNumber("Y offset degrees", m_vision.getVerticalOffsetDegrees()); 
    SmartDashboard.putNumber("Area", m_vision.getTargetAreaPercentage());
    SmartDashboard.putNumber("Latency", m_vision.getLatency());
    SmartDashboard.putNumberArray("Robot pose", pose);
    //if the above line doesn't display the code to smart dashboard, try this line
    //SmartDashboard.putNumberArray("Botpose", m_vision.getRobotPose()); 
    SmartDashboard.putBoolean("Valid target detected", m_vision.validTargetDetected());
    SmartDashboard.putBoolean("2 valid targets detected", m_vision.twoValidTargetsDetected());
 
    // Print data
    System.out.println("X offset degrees: " + m_vision.getHorizontalOffsetDegrees()); 
    System.out.println("Y offset degrees: " + m_vision.getVerticalOffsetDegrees()); 
    System.out.println("Area: " + m_vision.getTargetAreaPercentage());
    System.out.println("Latency: " + m_vision.getLatency());
    System.out.printf("Robot pose: %.2f, %.2f at %.2f degrees\n", pose[0], pose[1], pose[5]);
    System.out.println("Valid target detected: "+m_vision.validTargetDetected());
    System.out.println("2 valid targets detected: "+m_vision.twoValidTargetsDetected());
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return false; 
  }


}

