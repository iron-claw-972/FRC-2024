package frc.robot.commands.vision;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.miscConstants.VisionConstants;
import frc.robot.util.DetectedObject;
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
    double[] xOffset = m_vision.getHorizontalOffset();
    double[] yOffset = m_vision.getVerticalOffset();
    String[] objectClass = m_vision.getDetectedObjectClass();

    //put the offsets and area on SmartDashboard for testing 
    SmartDashboard.putNumberArray("Object X offsets degrees", xOffset); 
    SmartDashboard.putNumberArray("Object Y offsets degrees", yOffset); 
    SmartDashboard.putNumberArray("Object Distances", m_vision.getDistance()); 

    DetectedObject bestGamePiece = m_vision.getBestGamePiece(40);
    SmartDashboard.putString("Vision best game piece", bestGamePiece.toString());
    System.out.println("Best game piece: "+bestGamePiece);
    for(int i = 0; i < xOffset.length; i++){
      System.out.printf("\nx: %.2f, y: %.2f, type: %s\n", xOffset[i], yOffset[i], objectClass[i]);
      DetectedObject object = new DetectedObject(xOffset[i], yOffset[i], objectClass[i], VisionConstants.CAMERAS.get(0).getSecond());
      System.out.printf("Object: %sDistance: %.2f, Angle: %.2f\n", object, object.getDistance(), Units.radiansToDegrees(object.getAngle()));
    }
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

