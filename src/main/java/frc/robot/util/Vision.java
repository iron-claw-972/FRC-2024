package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.vision.ReturnData;
import frc.robot.commands.vision.TakeSnapshots;
import frc.robot.commands.vision.TestVisionDistance;
import frc.robot.commands.vision.AimAtTag;
import frc.robot.commands.vision.CalculateStdDevs;
import frc.robot.commands.vision.ChaseTag;
import frc.robot.constants.Constants;
import frc.robot.constants.miscConstants.FieldConstants;
import frc.robot.subsystems.Drivetrain;

import java.util.ArrayList;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
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
  private NetworkTableEntry m_snapshot;
  private NetworkTableEntry m_tid;
  private NetworkTableEntry m_cameraPose;

  private ArrayList<double[]> m_yaws;

  /**
   * Creates a new instance of Vision and sets up the limelight NetworkTable and the SmartDashboard
   */
  public Vision(ShuffleboardTab shuffleboardTab) {
    m_shuffleboardTab = shuffleboardTab;

    //get the limelight table from the default NetworkTable instance
    m_visionTable = NetworkTableInstance.getDefault().getTable("limelight");

    m_tvDebouncer = new Debouncer(0.05, DebounceType.kRising);

    //from the table, get various entries that contain data 
    m_tv = m_visionTable.getEntry("tv"); 
    m_tx = m_visionTable.getEntry("tx"); 
    m_ty = m_visionTable.getEntry("ty"); 
    m_ta = m_visionTable.getEntry("ta"); 
    m_tl = m_visionTable.getEntry("tl"); 
    m_cl = m_visionTable.getEntry("cl"); 
    m_robotPoseVision = m_visionTable.getEntry("botpose_wpiblue");
    m_snapshot = m_visionTable.getEntry("snapshot");
    m_tid = m_visionTable.getEntry("tid");
    m_cameraPose = m_visionTable.getEntry("camerapose_robotspace");

    m_yaws = new ArrayList<double[]>();
    m_yaws.add(new double[]{0, 0});
  }

  /**
   * Pipeline 0 detects an april tag with an ID of 0, pipeline 1 detects an april tag with an ID of 1
   * @param pipeline
   */
  public void switchPipelines(int pipeline){
    m_visionTable.getEntry("pipeline").setNumber(pipeline); 
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
    return m_tvDebouncer.calculate(m_tv.getDouble(0)==1); 
  }

  /**
   * Returns the robot pose as a double array
   * @return A double array with x, y, z, roll, pitch, yaw
   */
  public double[] getRobotPose(){
    double[] pose = m_robotPoseVision.getDoubleArray(new double[6]);
    if(Constants.kLogging){
      LogManager.addDoubleArray("Vision/pose", pose);
    }
    return pose;
  }

  /**
   * Returns the robot pose as a double array calculated using the target height and camera position
   * @param yaw The yaw of the robot to use in the calculation (radians)
   * @return A double array with x, y, z, roll, pitch, yaw
   */
  public double[] getRobotPose(double yaw){
    int id = (int) m_tid.getDouble(0)-1;
    if(id<=0||id>FieldConstants.kAprilTags.size()){
      return new double[6];
    }
    Pose3d target = FieldConstants.kAprilTags.get(id).pose;
    double[] camera = m_cameraPose.getDoubleArray(new double[6]);
    double fieldRelativeAngle = yaw+Units.degreesToRadians(camera[5]-getHorizontalOffsetDegrees());
    double verticalAngle = Units.degreesToRadians(camera[4]+getVerticalOffsetDegrees());
    double dist = (target.getZ()-camera[2])/Math.tan(verticalAngle);
    double x = target.getX()-Math.cos(fieldRelativeAngle)*dist;
    double y = target.getY()-Math.sin(fieldRelativeAngle)*dist;
    x -= camera[0]*Math.cos(yaw) + camera[1]*Math.sin(yaw);
    y -= camera[0]*Math.sin(yaw) + camera[1]*Math.cos(yaw);
    double[] pose = new double[]{x, y, 0, 0, 0, yaw};
    if(Constants.kLogging){
      LogManager.addDoubleArray("Vision/pose", pose);
    }
    return pose;
  }

  /**
   * Returns the estimated position as a Pose2d
   * @return a Pose2d
   */
  public Pose2d getPose2d(){
    if(validTargetDetected()){
      double[] pose = getRobotPose();
      if(pose[0] >= 0 || pose[1] >= 0){
        return null;
      }
      return new Pose2d(pose[0], pose[1], Rotation2d.fromDegrees(pose[5]));
    }else{
      return null;
    }
  }

  /**
   * Returns the estimated position as a Pose2d calculated using the target height and camera position
   * @param useManualCalculation If it should use manual calculations (recommended true)
   * @return a Pose2d
   */
  public Pose2d getPose2d(boolean useManualCalculation){
    if(!useManualCalculation){
      return getPose2d();
    }
    if(validTargetDetected()){
      updateYaws();
      double[] pose = getRobotPose(m_yaws.get(0)[0]);
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
   * Returns the Pose2d and the time stamp in seconds calculated using the target height and camera position
   * @param useManualCalculation If it should use manual calculations (recommended true)
   * @return a pair with a Pose2d and double
   */
  public Pair<Pose2d, Double> getPose2dWithTimeStamp(boolean useManualCalculation){
    return new Pair<Pose2d, Double>(getPose2d(useManualCalculation), getTimeStamp());
  }

  public double getTimeStamp(){
    return Timer.getFPGATimestamp()-getLatency()/1000;
  }

  /**
   * Set up the vision commands on SmartDashboard so we can turn them on/off for testing
   */
  public void setUpSmartDashboardCommandButtons(Drivetrain drive){
    SmartDashboard.putData("Vision ReturnData command", new ReturnData(this));
    m_shuffleboardTab.add("Return Data", new ReturnData(this));
    SmartDashboard.putData("Calculate vision std devs", new CalculateStdDevs(1000, this));
    m_shuffleboardTab.add("Calculate std devs", new CalculateStdDevs(1000, this));
    SmartDashboard.putData("Vision aim at tag", new AimAtTag(drive));
    m_shuffleboardTab.add("Aim at tag", new AimAtTag(drive));
    SmartDashboard.putData("Vision aim at tag 2", new ChaseTag(drive, this));
    m_shuffleboardTab.add("Aim at tag 2", new ChaseTag(drive, this));
    SmartDashboard.putData("Vision distance test (forward)", new TestVisionDistance(0.1, drive, this));
    m_shuffleboardTab.add("Distance test (forward)", new TestVisionDistance(0.1, drive, this));
    SmartDashboard.putData("Vision distance test (backward)", new TestVisionDistance(-0.1, drive, this));
    m_shuffleboardTab.add("Distance test (backward)", new TestVisionDistance(-0.1, drive, this));
    SmartDashboard.putData("Take vision snapshots", new TakeSnapshots(m_snapshot));
    m_shuffleboardTab.add("Take snapshots", new TakeSnapshots(m_snapshot));
  }

  public void addYaw(double yaw){
    m_yaws.add(new double[]{yaw, Timer.getFPGATimestamp()});
    updateYaws();
  }

  public void updateYaws(){
    if(m_yaws.size() > 1 && m_yaws.get(1)[1] < getTimeStamp()){
      m_yaws.remove(0);
    }
  }
}
