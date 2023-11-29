package frc.robot.commands.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.Vision;

/**
 * Adds data from object detection vision to SmartDashboard
 */
public class ReturnData extends CommandBase{
  private final Vision m_vision; 

  /**
   * Adds data from object detection vision to Smartdashboard
   * @param vision The vision
   */
  public ReturnData(Vision vision){
    m_vision = vision;
  }
  
  /**
   * Adds the data to SmartDashboard
   */
  @Override
  public void execute() {

    //put the offsets and area on SmartDashboard for testing 
    SmartDashboard.putNumberArray("Object X offsets degrees", m_vision.getHorizontalOffset()); 
    SmartDashboard.putNumberArray("Object Distances", m_vision.getDistance()); 

    
  }

  /**
   * Does nothing
   * @param interrupted If the command is interrupted
   */
  @Override
  public void end(boolean interrupted) {

  }

  /**
   * Returns if the command is finished
   * @retrun Always false (command never finishes)
   */
  @Override
  public boolean isFinished() {
    return false; 
  }


}

