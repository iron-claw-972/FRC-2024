package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.vision.AcquireGamePiece;
import frc.robot.commands.vision.AcquireGamePiecePID;
import frc.robot.commands.vision.ReturnData;
import frc.robot.constants.miscConstants.VisionConstants;
import frc.robot.subsystems.Drivetrain;

// Vision and it's commands are adapted from Iron Claw's FRC2022, FRC2023, and: https://www.youtube.com/watch?v=TG9KAa2EGzQ&t=1439s
public class Vision {

  private ShuffleboardTab m_shuffleboardTab;

  private NetworkTable m_objectDetectionTable;

  // DoubleSubscribers for the subscribing to the topics with data
  private NetworkTableEntry m_xOffset;
  private NetworkTableEntry m_yOffset;
  private NetworkTableEntry m_objectDistance;
  private NetworkTableEntry m_objectClass;
  private NetworkTableEntry m_cameraIndex;
  private NetworkTableEntry m_objectDetected;

  /**
   * Creates a new instance of Vision and sets up the limelight NetworkTable and the SmartDashboard
   */
  public Vision(ShuffleboardTab shuffleboardTab) {
    m_shuffleboardTab = shuffleboardTab;

    // Initialize object_detection NetworkTable
    m_objectDetectionTable = NetworkTableInstance.getDefault().getTable("object_detection");

    // From the object_detection NetworkTable, get the entries
    m_objectDetected = m_objectDetectionTable.getEntry("object_detected");
    m_objectDistance = m_objectDetectionTable.getEntry("object_distance");
    m_xOffset = m_objectDetectionTable.getEntry("tx");
    m_yOffset = m_objectDetectionTable.getEntry("ty");
    //TODO: Change these to whatever the actual entry names are
    m_objectClass = m_objectDetectionTable.getEntry("class");
    m_cameraIndex = m_objectDetectionTable.getEntry("camera_index");

    // Start NetworkTables server
    NetworkTableInstance.getDefault().startServer();
  }

  /**
   * Get the horizontal offsets from the crosshair to the targets
   * @return An array of offsets in degrees
   */
  public double[] getHorizontalOffset(){
    return m_xOffset.getDoubleArray(new double[0]);
  }

  /**
   * Get the vertical offsets from the crosshair to the targets
   * @return An array of offsets in degrees
   */
  public double[] getVerticalOffset(){
    return m_yOffset.getDoubleArray(new double[0]);
  }

  /**
   * Get the target distances
   * @return Distance in meters
   */
  public double[] getDistance(){
    return m_objectDistance.getDoubleArray(new double[0]);
  }

  /**
   * Returns whether or not a valid object is detected
   * @return true or false
   */
  public boolean validObjectDetected(){
    return m_objectDetected.getBoolean(false);
  }

  /**
   * Returns what types of object are detected
   * @return The object types as a String array
   */
  public String[] getDetectedObjectClass(){
    return m_objectClass.getStringArray(new String[0]);
  }

  /**
   * Gets the camera indices (which camera sees the object)
   * @return The indices as a long array (method returns long array instead of int array)
   */
  public long[] getCameraIndex(){
    return m_cameraIndex.getIntegerArray(new long[0]);
  }

  /**
   * Stores all of the detected objects in an array
   * @return The array of DetectedObjects
   */
  public DetectedObject[] getDetectedObjects(){
    double[] xOffset = getHorizontalOffset();
    double[] yOffset = getVerticalOffset();
    double[] distance = getDistance();
    String[] objectClass = getDetectedObjectClass();
    long[] cameraIndex = getCameraIndex();
    DetectedObject[] objects = new DetectedObject[xOffset.length];
    for(int i = 0; i < objects.length; i++){
      objects[i] = new DetectedObject(
        Units.degreesToRadians(xOffset[i]),
        Units.degreesToRadians(yOffset[i]),
        distance[i],
        objectClass[i],
        VisionConstants.kCameras.get((int)cameraIndex[i]).getSecond()
      );
    }
    return objects;
  }

  /**
   * Returns the closest object in front of the robot
   * @param maxAngle The maximum angle from the front of the robot to use
   * @return The best DetectedObject
   */
  public DetectedObject getBestObject(double maxAngle){
    DetectedObject[] objects = getDetectedObjects();
    DetectedObject best = null;
    double closest = Double.POSITIVE_INFINITY;
    for(DetectedObject object : objects){
      double dist = object.getDistance();
      if(Math.abs(object.getRelativeAngle()) < maxAngle && dist < closest){
        closest = dist;
        best = object;
      }
    }
    return best;
  }

  /**
   * Set up the vision commands on SmartDashboard so we can turn them on/off for testing
   */
  public void setUpSmartDashboardCommandButtons(Drivetrain drive){
    m_shuffleboardTab.add("Return Data", new ReturnData(this));
    SmartDashboard.putData("Vision Return Data", new ReturnData(this));
    m_shuffleboardTab.add("Acquire Game Piece PID", new AcquireGamePiecePID(drive, this));
    SmartDashboard.putData("Acquire Game Piece PID", new AcquireGamePiecePID(drive, this));
    m_shuffleboardTab.add("Acquire Game Piece", new AcquireGamePiece(()->getBestObject(60), drive));
    SmartDashboard.putData("Acquire Game Piece", new AcquireGamePiece(()->getBestObject(60), drive));
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
