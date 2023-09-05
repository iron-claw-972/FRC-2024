package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.vision.ReturnData;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

//Vision and it's commands are adapted from Iron Claw's FRC2022 and: https://www.youtube.com/watch?v=TG9KAa2EGzQ&t=1439s
public class Vision {

  NetworkTable m_visionTable; 

  Debouncer m_tvDebouncer; 

  NetworkTableEntry m_tv;
  NetworkTableEntry m_tx;
  NetworkTableEntry m_ty;
  NetworkTableEntry m_ta;
  NetworkTableEntry m_robotPoseVision; 

  //default array of robot pose. TODO: Check if this is the right way of doing this. 
  double[] array = {0.0};


  public Vision() {
    //get the limelight table from the default NetworkTable instance
    m_visionTable = NetworkTableInstance.getDefault().getTable("limelight");

    m_tvDebouncer = new Debouncer(0.05, DebounceType.kBoth);

    //from the table, get various entries that contain data 
    m_tv = m_visionTable.getEntry("tv"); 
    m_tx = m_visionTable.getEntry("tx"); 
    m_ty = m_visionTable.getEntry("ty"); 
    m_ta = m_visionTable.getEntry("ta"); 
    m_robotPoseVision = m_visionTable.getEntry("botpose"); 

    //set up the vision commands on SmartDashboard so we can turn them on/off for testing
    setUpSmartDashboardCommandButtons();
  }


  /**
   * Get the horizontal offset in degrees from the crosshair to the target
   * @return offset in degrees
   */
  public double getHorizontalOffsetDegrees(){
    return m_tx.getDouble(0.0);
  }

  /**
   * Get the vertical offset in degrees from the crosshair to the target
   * @return offset in degrees
   */
  public double getVerticalOffsetDegrees(){
    return m_ty.getDouble(0.0);
  }

  /**
   * Get the target area (percentage of the image[screen?] that the target takes up)
   * @return percentage
   */
  public double getTargetAreaPercentage(){
    return m_ta.getDouble(0.0);
  }

  /**
   * Returns whether or not a valid target was detected after being passed through a debouncer 
   * to make sure that we are really locked on the target. We wait 0.05 seconds and check whether or not we are still locked on our target. 
   * 
   * @return true or false 
   */
  public boolean validTargetDetected(){
    return m_tvDebouncer.calculate(m_tv.getBoolean(false)); 
  }

  /**
   * Returns whether or not a valid target was detected after being passed through a debouncer 
   * to make sure that we are really locked on the target. We wait 0.05 seconds and check whether or not we are still locked on our target. 
   * This requires some setup in Limelight. https://docs.limelightvision.io/en/latest/apriltags_in_3d.html
   * 
   * @return true or false 
   */
  public double[] getRobotPoseVision(){
    
    return m_robotPoseVision.getDoubleArray(array); 
  }

  /**
   * Set up the vision commands on SmartDashboard so we can turn them on/off for testing
   */
  public void setUpSmartDashboardCommandButtons(){
    SmartDashboard.putData("ReturnData command", new ReturnData(this));

  }

  
}
