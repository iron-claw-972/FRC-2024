package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

// Vision and it's commands are adapted from Iron Claw's FRC2022, FRC2023, and: https://www.youtube.com/watch?v=TG9KAa2EGzQ&t=1439s
public class Vision {

  private ShuffleboardTab m_shuffleboardTab;

  private NetworkTable m_objectDetectionTable;

  // DoubleSubscribers for the subscribing to the topics with data
  private DoubleSubscriber m_tx;
  private DoubleSubscriber m_ty;
  private DoubleSubscriber m_objectDistance;
  private BooleanSubscriber m_objectDetected;

  /**
   * Creates a new instance of Vision and sets up the limelight NetworkTable and the SmartDashboard
   */
  public Vision(ShuffleboardTab shuffleboardTab) {
    m_shuffleboardTab = shuffleboardTab;

    m_objectDetectionTable = NetworkTableInstance.getDefault().getTable("object_detection");

    // From the object_detection NetworkTable, subscribe to the various topics with data
    m_objectDetected = m_objectDetectionTable.getBooleanTopic("object_detected").subscribe(false);
    m_objectDistance = m_objectDetectionTable.getDoubleTopic("object_distance").subscribe(0.0);
    m_tx = m_objectDetectionTable.getDoubleTopic("tx").subscribe(0.0);
    m_ty = m_objectDetectionTable.getDoubleTopic("ty").subscribe(0.0);

    //set up the vision commands on SmartDashboard so we can turn them on/off for testing
    setUpSmartDashboardCommandButtons();
  }

  public Vision(){} //empty constructor for use of vision without shuffleboard/smartdashboard

  /**
   * Get the horizontal offset from the crosshair to the target
   * @return offset in degrees
   */
  public double getHorizontalOffset(){
    //TODO: Add this
    // It might be better to return this (and almost everything else) as an array, depending on how the dtection works
    return m_tx.get();
  }

  /**
   * Get the vertical offset from the crosshair to the target
   * @return offset in degrees
   */
  public double getVerticalOffset(){
    //TODO: Add this
    return m_ty.get();
  }

  /**
   * Get the target distance
   * @return Distance in meters
   */
  public double getDistance(){
    //TODO: Add this
    return m_objectDistance.get();
  }

  /**
   * Returns the total latency in ms
   * @return the latency as a double
   */
  public double getLatency(){
    //TODO: Add this, or delete it if it's unnecessary
    return 0;
  }

  /**
   * Returns whether or not a valid object is detected
   * @return true or false
   */
  public boolean validObjectDetected(){
    //TODO: Add this
    return m_objectDetected.get();
  }

  /**
   * Returns whether or not a valid object is detected
   * @return true or false 
   */
  public String returnDetectedObjectClass(){
    //TODO: Add this

    if(validObjectDetected()){
      return "hello";
    }
    return null; 
  }


  /**
   * Set up the vision commands on SmartDashboard so we can turn them on/off for testing
   */
  public void setUpSmartDashboardCommandButtons(){
    //TODO: Add this
  }

  /**
   * Prevents errors in CalculateStdDevs command
   * TODO: Delete this when merging with April tags
   * @return null
   */
  public Pose2d getPose2d(){return null;}
  
  /**
   * Prevents errors in CalculateStdDevs command
   * TODO: Delete this when merging with April Tags
   * @return 0.0
   */
  public double getTargetAreaPercentage(){return 0.0;}
}
