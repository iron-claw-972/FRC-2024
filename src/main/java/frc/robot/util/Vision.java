package frc.robot.util;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.vision.AimAtTag;
import frc.robot.commands.vision.AlignToTag;
import frc.robot.commands.vision.CalculateStdDevs;
import frc.robot.commands.vision.TestVisionDistance;
import frc.robot.constants.Constants;
import frc.robot.constants.miscConstants.FieldConstants;
import frc.robot.constants.miscConstants.VisionConstants;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.subsystems.Drivetrain;

// Vision and it's commands are adapted from Iron Claw's FRC2022, FRC2023, and: https://www.youtube.com/watch?v=TG9KAa2EGzQ&t=1439s
public class Vision {

  private ShuffleboardTab m_shuffleboardTab;

  // The field layout
  private AprilTagFieldLayout m_aprilTagFieldLayout;
  // A list of the cameras on the robot
  private ArrayList<VisionCamera> m_cameras = new ArrayList<>();

  /**
   * Creates a new instance of Vision and sets up the limelight NetworkTable and the SmartDashboard
   */
  public Vision(ShuffleboardTab shuffleboardTab, ArrayList<Pair<String, Transform3d>> camList) {
    m_shuffleboardTab = shuffleboardTab;
    
    try {
      // Try to find the field layout
      m_aprilTagFieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
    } catch (IOException e) {
      // If it can't find it, use the layout in the constants
      m_aprilTagFieldLayout = new AprilTagFieldLayout(FieldConstants.APRIL_TAGS, FieldConstants.kFieldLength, FieldConstants.kFieldWidth);
      DriverStation.reportWarning("Could not find k2023ChargedUp.m_resourceFile, check that GradleRIO is updated to at least 2023.2.1 in build.gradle",  e.getStackTrace());
      System.out.println("Could not find k2023ChargedUp.m_resourceFile, check that GradleRIO is updated to at least 2023.2.1 in build.gradle");
    }
    // Sets the origin to the right side of the blue alliance wall
    m_aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

    // Puts the cameras in an array list
    for (int i = 0; i < camList.size(); i++) {
      m_cameras.add(new VisionCamera(camList.get(i).getFirst(), camList.get(i).getSecond()));
    }
  }

  /**
   * Set up the vision commands on SmartDashboard so we can turn them on/off for testing
   */
  public void setUpSmartDashboardCommandButtons(Drivetrain drive){
    SmartDashboard.putData("Calculate vision std devs", new CalculateStdDevs(1000, this, drive));
    m_shuffleboardTab.add("Calculate std devs", new CalculateStdDevs(1000, this, drive));
    SmartDashboard.putData("Vision aim at tag", new AimAtTag(drive));
    m_shuffleboardTab.add("Aim at tag", new AimAtTag(drive));
    SmartDashboard.putData("Vision distance test (forward)", new TestVisionDistance(0.1, drive, this));
    m_shuffleboardTab.add("Distance test (forward)", new TestVisionDistance(0.1, drive, this));
    SmartDashboard.putData("Vision distance test (backward)", new TestVisionDistance(-0.1, drive, this));
    m_shuffleboardTab.add("Distance test (backward)", new TestVisionDistance(-0.1, drive, this));
    SmartDashboard.putData("Vision align to tag", new AlignToTag(drive));
    m_shuffleboardTab.add("Align to tag", new AlignToTag(drive));
  }

  /**
   * Gets the pose as a Pose2d using PhotonVision
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
    
    if (estimatedPoses.size() == 1) return estimatedPoses.get(0).estimatedPose.toPose2d();
    
    if (estimatedPoses.size() == 2) {
      return new Pose2d(
        estimatedPoses.get(0).estimatedPose.toPose2d().getTranslation()
          .plus(estimatedPoses.get(1).estimatedPose.toPose2d().getTranslation())
          .div(2),
        
        new Rotation2d(MathUtils.modulusMidpoint(
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
    if(id < 1 || id > getAprilTagFieldLayout().getTags().size()){
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
      if(VisionConstants.USE_MANUAL_CALCULATIONS){
        Pose2d pose = m_cameras.get(i).getEstimatedPose(referencePose.getRotation().getRadians());
        if(pose != null){
          EstimatedRobotPose estimatedPose = new EstimatedRobotPose(
            new Pose3d(pose.getX(), pose.getY(), 0, new Rotation3d(0, 0, pose.getRotation().getRadians())), 
            m_cameras.get(i).getTimeStamp(), 
            List.of(m_cameras.get(i).getBestTarget())
          );
          estimatedPoses.add(estimatedPose);
          if(Constants.kLogging){
            LogManager.addDoubleArray("Vision/camera " + i + "/estimated pose2d", new double[] {
              pose.getX(),
              pose.getY(),
              pose.getRotation().getRadians()
            });
          }
        }
      }else{
        Optional<EstimatedRobotPose> estimatedPose = m_cameras.get(i).getEstimatedPose(referencePose);
        // If the camera can see an april tag that exists, add it to the array list
        // April tags that don't exist might return a result that is present but doesn't have a pose
        if (estimatedPose.isPresent() && estimatedPose.get().estimatedPose != null) {
          estimatedPoses.add(estimatedPose.get());
          if(Constants.kLogging){
            LogManager.addDoubleArray("Vision/camera " + i + "/estimated pose2d", new double[] {
              estimatedPose.get().estimatedPose.getX(),
              estimatedPose.get().estimatedPose.getY(),
              estimatedPose.get().estimatedPose.getRotation().getZ()
            });
          }
        }
      }
    }
    return estimatedPoses;
  }

  /**
   * Updates the robot's odometry with vision
   * @param poseEstimator The pose estimator to update
   */
  public void updateOdometry(SwerveDrivePoseEstimator poseEstimator){
    // An array list of poses returned by different cameras
    ArrayList<EstimatedRobotPose> estimatedPoses = getEstimatedPoses(poseEstimator.getEstimatedPosition());
    for (int i = 0; i < estimatedPoses.size(); i++) {
      EstimatedRobotPose estimatedPose = estimatedPoses.get(i);
      // Continue if this pose doesn't exist
      if(estimatedPose==null || estimatedPose.estimatedPose==null || estimatedPose.timestampSeconds < 0){
        continue;
      }
      // Adds the vision measurement for this camera
      poseEstimator.addVisionMeasurement(
        estimatedPose.estimatedPose.toPose2d(),
        estimatedPose.timestampSeconds,
        VisionConstants.VISION_STD_DEVS
      );
    }
  }
  
  private class VisionCamera {
    PhotonCamera camera;
    PhotonPoseEstimator photonPoseEstimator;
    Pose2d lastPose;
    double lastTimestamp = 0;
  
    /**
     * Stores information about a camera
     * @param cameraName The name of the camera on PhotonVision
     * @param robotToCam The transformation from the robot to the camera
     */
    public VisionCamera(String cameraName, Transform3d robotToCam) {
      camera = new PhotonCamera(cameraName);
      photonPoseEstimator = new PhotonPoseEstimator(
        m_aprilTagFieldLayout, 
        PoseStrategy.AVERAGE_BEST_TARGETS, 
        camera, 
        robotToCam
      );
      photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
      photonPoseEstimator.setReferencePose(new Pose2d());
      lastPose = null;
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
          if (targetsUsed.get(i).getPoseAmbiguity() > VisionConstants.HIGHEST_AMBIGUITY) {
            return Optional.empty();
          }
        }
      }

      Optional<EstimatedRobotPose> pose = photonPoseEstimator.update(cameraResult);
      
      if(pose.isPresent() && pose.get()!=null && pose.get().estimatedPose!=null){
        double timestamp = getTimeStamp();
        if(lastPose==null || lastPose.getTranslation().getDistance(pose.get().estimatedPose.toPose2d().getTranslation())>DriveConstants.kMaxSpeed*(timestamp-lastTimestamp)){
          lastPose = pose.get().estimatedPose.toPose2d();
          lastTimestamp = timestamp;
          return Optional.empty();
        }
        lastPose = pose.get().estimatedPose.toPose2d();
        lastTimestamp = timestamp;
      }

      return pose;
    }
    
    /**
     * Gets the pose using manual calculations
     * @param yaw The yaw of the robot to use in the calculation
     * @return estimated pose as a Pose2d
     */
    public Pose2d getEstimatedPose(double yaw){
      PhotonTrackedTarget target = camera.getLatestResult().getBestTarget();
      if(target==null){
        return null;
      }
      int id = target.getFiducialId();
      if(id<=0||id>FieldConstants.APRIL_TAGS.size()){
        return null;
      }
      Pose3d targetPose = FieldConstants.APRIL_TAGS.get(id).pose;
      Transform3d robotToCamera = photonPoseEstimator.getRobotToCameraTransform();
      // Get the tag position relative to the robot, assuming the robot is on the ground
      Translation3d translation = new Translation3d(1, new Rotation3d(0, target.getPitch(), target.getYaw()));
      translation = translation.rotateBy(robotToCamera.getRotation());
      translation = translation.times(translation.getZ()/(targetPose.getZ()-robotToCamera.getZ()));
      translation = translation.plus(robotToCamera.getTranslation());
      translation = translation.rotateBy(new Rotation3d(
        0, 0, yaw
      ));
      // Invert it to get the robot position relative to the camera
      translation = translation.times(-1);
      // Get the field relative pose
      translation = translation.plus(targetPose.getTranslation());
      return new Pose2d(translation.toTranslation2d(), new Rotation2d(yaw));
    }

    /**
     * Gets the last timestamp in seconds
     * @return The timestamp in seconds
     */
    public double getTimeStamp(){
      return camera.getLatestResult().getTimestampSeconds();
    }
    
    /**
     * Gets the best target
     * @return A PhotonTrackedTarget
     */
    public PhotonTrackedTarget getBestTarget(){
      return camera.getLatestResult().getBestTarget();
    }
  }
}
