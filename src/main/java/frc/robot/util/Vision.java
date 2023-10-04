package frc.robot.util;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.apache.commons.lang3.Functions;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.vision.AimAtTag;
import frc.robot.commands.vision.CalculateStdDevs;
import frc.robot.commands.vision.ChaseTag;
import frc.robot.commands.vision.ReturnData;
import frc.robot.commands.vision.TakeSnapshots;
import frc.robot.commands.vision.TestVisionDistance;
import frc.robot.constants.Constants;
import frc.robot.constants.miscConstants.FieldConstants;
import frc.robot.constants.miscConstants.VisionConstants;
import frc.robot.subsystems.Drivetrain;

// Vision and it's commands are adapted from Iron Claw's FRC2022, FRC2023, and: https://www.youtube.com/watch?v=TG9KAa2EGzQ&t=1439s
public class Vision {

  private ShuffleboardTab m_shuffleboardTab;

  private Debouncer m_debouncer;

  private ArrayList<double[]> m_yaws;

  // The field layout
  private AprilTagFieldLayout m_aprilTagFieldLayout;
  // A list of the cameras on the robot
  private ArrayList<VisionCamera> m_cameras = new ArrayList<>();

  /**
   * Creates a new instance of Vision and sets up the limelight NetworkTable and the SmartDashboard
   */
  public Vision(ShuffleboardTab shuffleboardTab, ArrayList<Pair<String, Transform3d>> camList) {
    m_shuffleboardTab = shuffleboardTab;

    m_debouncer = new Debouncer(0.05, DebounceType.kRising);
    
    try {
      // Try to find the field layout
      m_aprilTagFieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
    } catch (IOException e) {
      // If it can't find it, use the layout in the constants
      m_aprilTagFieldLayout = new AprilTagFieldLayout(FieldConstants.kAprilTags, FieldConstants.kFieldLength, FieldConstants.kFieldWidth);
      DriverStation.reportWarning("Could not find k2023ChargedUp.m_resourceFile, check that GradleRIO is updated to at least 2023.2.1 in build.gradle",  e.getStackTrace());
    }
    // Sets the origin to the right side of the blue alliance wall
    m_aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

    // Puts the cameras in an array list
    for (int i = 0; i < camList.size(); i++) {
      m_cameras.add(new VisionCamera(camList.get(i).getFirst(), camList.get(i).getSecond()));
    }
    
    m_yaws = new ArrayList<double[]>();
    m_yaws.add(new double[]{0, 0});
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
    return m_debouncer.calculate(m_tv.getDouble(0)==1); 
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
      LogManager.addDouble("Vision/tag dist", dist);
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
      if(pose[0] < 0 || pose[1] < 0){
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
      if(pose[0] < 0 || pose[1] < 0){
        return null;
      }
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

  /**
   * Gets the pose as a Pose2d
   * @param referencePoses The reference poses in order of preference, null poses will be skipped
   * @return The pose of the robot, or null if it can't see april tags
   */
  public Pose2d getPose2d(Pose2d... referencePoses){
    Pose2d referencePose = new Pose2d();
    for (Pose2d checkReferencePose:referencePoses){
      if (checkReferencePose != null) {
        referencePose = checkReferencePose;
        break;
      }
    }
    ArrayList<EstimatedRobotPose> estimatedPoses = getEstimatedPoses(referencePose);
    Translation2d translation = new Translation2d();
    double rotation = 0;
    
    if (estimatedPoses.size() == 1) return estimatedPoses.get(0).estimatedPose.toPose2d();
    
    if (estimatedPoses.size() == 2) {
      return new Pose2d(
        estimatedPoses.get(0).estimatedPose.toPose2d().getTranslation()
          .plus(estimatedPoses.get(1).estimatedPose.toPose2d().getTranslation())
          .div(2),
        
        new Rotation2d(Functions.modulusMidpoint(
          estimatedPoses.get(0).estimatedPose.toPose2d().getRotation().getRadians(),
          estimatedPoses.get(1).estimatedPose.toPose2d().getRotation().getRadians(),
          -Math.PI, Math.PI)
        )
      );
    }
          
    //TODO: VERY LOW PRIORITY FOR FUTURE ROBOTS, make the rotation average work with more than 2 cameras
    // for(int i = 0; i < estimatedPoses.size(); i ++){
    //   translation=translation.plus(estimatedPoses.get(i).estimatedPose.toPose2d().getTranslation());
    // }

    // if(posesUsed>0){
    //   return new Pose2d(translation.div(estimatedPoses.size()), new Rotation2d());
    // }
    return null;
  }

  public AprilTagFieldLayout getAprilTagFieldLayout(){
    return m_aprilTagFieldLayout;
  }

  /**
   * Gets the pose of an april tag
   * @param id AprilTag id (1-8)
   * @return Pose3d of the AprilTag
   */
  public Pose3d getTagPose(int id){
    if(id < 1 || id > 8){
      System.out.println("Tried to find the pose of april tag "+id);
      return null;
    }
    return getAprilTagFieldLayout().getTagPose(id).get();
  }

  /**
   * Returns where it thinks the robot is
   * @param referencePose The pose to use as a reference, usually the previous robot pose
   * @return An array list of estimated poses, one for each camera that can see an april tag
   */
  public ArrayList<EstimatedRobotPose> getEstimatedPoses(Pose2d referencePose) {
    ArrayList<EstimatedRobotPose> estimatedPoses = new ArrayList<>();
    for (int i = 0; i < m_cameras.size(); i++) {
      Optional<EstimatedRobotPose> estimatedPose = m_cameras.get(i).getEstimatedPose(referencePose);
      // If the camera can see an april tag that exists, add it to the array list
      // April tags that don't exist might return a result that is present but doesn't have a pose
      if (estimatedPose.isPresent() && estimatedPose.get().estimatedPose != null) {
        estimatedPoses.add(estimatedPose.get());
        LogManager.addDoubleArray("Vision/camera " + i + "/estimated pose2d", new double[] {
          estimatedPose.get().estimatedPose.getX(),
          estimatedPose.get().estimatedPose.getY(),
          estimatedPose.get().estimatedPose.getRotation().getZ()
        });
      }
    }
    return estimatedPoses;
  }
  
  private class VisionCamera {
    PhotonCamera camera;
    PhotonPoseEstimator photonPoseEstimator;
  
    /**
     * Stores information about a camera
     * @param cameraName The name of the camera on PhotonVision
     * @param robotToCam The transformation from the robot to the camera
     */
    public VisionCamera(String cameraName, Transform3d robotToCam) {
      camera = new PhotonCamera(cameraName);
      photonPoseEstimator = new PhotonPoseEstimator(
        m_aprilTagFieldLayout, 
        PoseStrategy.MULTI_TAG_PNP, 
        camera, 
        robotToCam
      );
      photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
      photonPoseEstimator.setReferencePose(new Pose2d());
      camera.setLED(VisionLEDMode.kOn);
    }
  
    /**
     * Gets the estimated pose from the camera
     * @param referencePose Pose to use for reference, usually the previous estimated robot pose
     * @return estimated robot pose
     */
    public Optional<EstimatedRobotPose> getEstimatedPose(Pose2d referencePose) {
      photonPoseEstimator.setReferencePose(referencePose);

      PhotonPipelineResult cameraResult = camera.getLatestResult();
      
      // if there is a target detected and not in the past, 
      // check the ambiguity isn't too high
      if (cameraResult.hasTargets() && cameraResult.getTimestampSeconds() > 0) {
        // go through all the targets
        List<PhotonTrackedTarget> targetsUsed = cameraResult.targets;
        for (int i = 0; i < targetsUsed.size(); i++) {
          // check their ambiguity, if it is above the highest wanted amount, return nothing
          if (targetsUsed.get(i).getPoseAmbiguity() > VisionConstants.highestAmbiguity) {
            return Optional.empty();
          }
        }
      }

      Optional<EstimatedRobotPose> pose = photonPoseEstimator.update(cameraResult);

      return pose;
    }
  }
}
