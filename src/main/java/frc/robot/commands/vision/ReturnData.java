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
    SmartDashboard.putNumber("X offset degrees", m_vision.getHorizontalOffset()); 
    SmartDashboard.putNumber("Distance", m_vision.getDistance()); 

    
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return false; 
  }


}

