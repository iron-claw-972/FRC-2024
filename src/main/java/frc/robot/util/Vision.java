package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.vision.ReturnData;
import frc.robot.constants.Constants;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

// Vision and it's commands are adapted from Iron Claw's FRC2022, FRC2023, and: https://www.youtube.com/watch?v=TG9KAa2EGzQ&t=1439s
public class Vision {

  private ShuffleboardTab m_shuffleboardTab;

  private NetworkTable m_visionTable; 

  private Debouncer m_tvDebouncer; 

  private NetworkTableEntry m_tv;
  private NetworkTableEntry m_tx;
  private NetworkTableEntry m_ty;
  private NetworkTableEntry m_ta;
  private NetworkTableEntry m_tl;
  private NetworkTableEntry m_cl;
  private NetworkTableEntry m_robotPoseVision; 

  /**
   * Creates a new instance of Vision and sets up the limelight NetworkTable and the SmartDashboard
   */
  public Vision(ShuffleboardTab shuffleboardTab) {
    m_shuffleboardTab = shuffleboardTab;

    //get the limelight table from the default NetworkTable instance
    m_visionTable = NetworkTableInstance.getDefault().getTable("limelight");

    m_tvDebouncer = new Debouncer(0.05, DebounceType.kBoth);

    //from the table, get various entries that contain data 
    m_tv = m_visionTable.getEntry("tv"); 
    m_tx = m_visionTable.getEntry("tx"); 
    m_ty = m_visionTable.getEntry("ty"); 
    m_ta = m_visionTable.getEntry("ta"); 
    m_tl = m_visionTable.getEntry("tl"); 
    m_cl = m_visionTable.getEntry("cl"); 
    m_robotPoseVision = m_visionTable.getEntry("botpose"); 

    //set up the vision commands on SmartDashboard so we can turn them on/off for testing
    setUpSmartDashboardCommandButtons();
  }


  /**
   * Get the horizontal offset in degrees from the crosshair to the target
   * @return offset in degrees
   */
  public double getHorizontalOffsetDegrees(){
    return m_tx.getDouble(0);
  }

  /**
   * Get the vertical offset in degrees from the crosshair to the target
   * @return offset in degrees
   */
  public double getVerticalOffsetDegrees(){
    return m_ty.getDouble(0);
  }

  /**
   * Get the target area (percentage of the image (screen) that the target takes up)
   * @return percentage
   */
  public double getTargetAreaPercentage(){
    return m_ta.getDouble(0);
  }

  /**
   * Returns the total latency in ms from the limelight
   * @return the latency as a double
   */
  public double getLatency(){
    return m_tl.getDouble(0)+m_cl.getDouble(0);
  }

  /**
   * Returns whether or not a valid target was detected after being passed through a debouncer 
   * to make sure that we are really locked on the target. We wait 0.05 seconds and check whether or not we are still locked on our target. 
   * @return true or false 
   */
  public boolean validTargetDetected(){
    return m_tvDebouncer.calculate(m_tv.getBoolean(false)); 
  }

  /**
   * Returns the robot pose as a double array
   * @return A double array with x, y, z, roll, pitch, yaw
   */
  public double[] getRobotPose(){
    //double[] pose = 
    // if(Constants.kLogging){
    //   LogManager.addDoubleArray("Vision/pose", pose);
    // }
    return m_robotPoseVision.getDoubleArray(new double[6]);
  }

  /**
   * Returns the estimated position as a Pose2d
   * @return a Pose2d
   */
  public Pose2d getPose2d(){
    if(validTargetDetected()){
      double[] pose = getRobotPose();
      return new Pose2d(pose[0], pose[1], Rotation2d.fromDegrees(pose[5]));
    }else{
      return null;
    }
  }

  /**
   * Returns the Pose2d and the time stamp in seconds
   * @return a pair with a Pose2d and double
   */
  public Pair<Pose2d, Double> getPose2dWithTimeStamp(){
    return new Pair<Pose2d, Double>(getPose2d(), Timer.getFPGATimestamp()-getLatency()/1000);
  }

  /**
   * Set up the vision commands on SmartDashboard so we can turn them on/off for testing
   */
  public void setUpSmartDashboardCommandButtons(){
    SmartDashboard.putData("Vision ReturnData command", new ReturnData(this));
    m_shuffleboardTab.add("Return Data", new ReturnData(this));
  }
}
