package frc.robot.commands.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.Vision;

public class ReturnBasicData extends CommandBase{
  private final Vision m_vision; 

  public ReturnBasicData(Vision vision){
    m_vision = vision;
  }
  
  @Override
  public void execute() {
    //put the offsets and area on SmartDashboard for testing 
    SmartDashboard.putNumber("X offset degrees", m_vision.getHorizontalOffsetDegrees()); 
    SmartDashboard.putNumber("Y offset degrees", m_vision.getVerticalOffsetDegrees()); 
    SmartDashboard.putNumber("Area", m_vision.getTargetAreaPercentage()); 
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return false; 
  }


}

