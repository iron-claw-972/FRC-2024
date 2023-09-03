package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.vision.ChaseTag;
import frc.robot.commands.vision.ReturnBasicData;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision {

  NetworkTable visionTable; 

  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;


  public Vision() {
    visionTable = NetworkTableInstance.getDefault().getTable("limelight");
    tx = visionTable.getEntry("tx"); 
    ty = visionTable.getEntry("ty"); 
    ta = visionTable.getEntry("ta"); 

    setUpSmartDashboardCommandButtons();
  }

  /**
   * Get the horizontal offset in degrees from the crosshair to the target
   * @return offset in degrees
   */
  public double getHorizontalOffsetDegrees(){
    return tx.getDouble(0.0);
  }

  /**
   * Get the vertical offset in degrees from the crosshair to the target
   * @return offset in degrees
   */
  public double getVerticalOffsetDegrees(){
    return ty.getDouble(0.0);
  }

  /**
   * Get the percentage of the image(screen?) that the target takes up
   * @return percentage
   */
  public double getTargetAreaPercentage(){
    return ta.getDouble(0.0);
  }

  public void setUpSmartDashboardCommandButtons(){
    SmartDashboard.putData("ChaseTag command", new ReturnBasicData(this));
  }

  
}
