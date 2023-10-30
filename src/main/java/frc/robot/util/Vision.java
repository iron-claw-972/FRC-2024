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
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.vision.AimAtTag;
import frc.robot.commands.vision.CalculateStdDevs;
import frc.robot.commands.vision.TestVisionDistance;
import frc.robot.constants.Constants;
import frc.robot.constants.miscConstants.FieldConstants;
import frc.robot.constants.miscConstants.VisionConstants;
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
      m_aprilTagFieldLayout = new AprilTagFieldLayout(FieldConstants.kAprilTags, FieldConstants.kFieldLength, FieldConstants.kFieldWidth);
      DriverStation.reportWarning("Could not find k2023ChargedUp.m_resourceFile, check that GradleRIO is updated to at least 2023.2.1 in build.gradle",  e.getStackTrace());
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
        if(Constants.kLogging){
          LogManager.addDoubleArray("Vision/camera " + i + "/estimated pose2d", new double[] {
            estimatedPose.get().estimatedPose.getX(),
            estimatedPose.get().estimatedPose.getY(),
            estimatedPose.get().estimatedPose.getRotation().getZ()
          });
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
      // Adds the vision measurement for this camera
      poseEstimator.addVisionMeasurement(
        estimatedPose.estimatedPose.toPose2d(),
        estimatedPose.timestampSeconds,
        VisionConstants.kVisionStdDevs
      );
    }
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
        PoseStrategy.LOWEST_AMBIGUITY, 
        camera, 
        robotToCam
      );
      photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
      photonPoseEstimator.setReferencePose(new Pose2d());
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
          if (targetsUsed.get(i).getPoseAmbiguity() > VisionConstants.kHighestAmbiguity) {
            return Optional.empty();
          }
        }
      }

      Optional<EstimatedRobotPose> pose = photonPoseEstimator.update(cameraResult);

      return pose;
    }
    
    /**
     * Gets the pose using manual calculations
     * @param yaw The yaw of the robot to use in the calculation
     * @return estimated pose as a Pose2d
     */
    @SuppressWarnings("unused")
    public Pose2d getEstimatedPose(double yaw){
      PhotonTrackedTarget target = camera.getLatestResult().getBestTarget();
      if(target==null){
        return null;
      }
      int id = target.getFiducialId();
      if(id<=0||id>FieldConstants.kAprilTags.size()){
        return null;
      }
      Pose3d targetPose = FieldConstants.kAprilTags.get(id).pose;
      Transform3d robotToCamera = photonPoseEstimator.getRobotToCameraTransform();
      double fieldRelativeAngle = yaw+robotToCamera.getRotation().getZ()-target.getYaw();
      double verticalAngle = robotToCamera.getRotation().getY()+target.getPitch();
      double dist = (targetPose.getZ()-robotToCamera.getZ())/Math.tan(verticalAngle);
      double x = targetPose.getX()-Math.cos(fieldRelativeAngle)*dist;
      double y = targetPose.getY()-Math.sin(fieldRelativeAngle)*dist;
      x -= robotToCamera.getX()*Math.cos(yaw) + robotToCamera.getY()*Math.sin(yaw);
      y -= robotToCamera.getX()*Math.sin(yaw) + robotToCamera.getY()*Math.cos(yaw);
      Pose2d pose = new Pose2d(x, y, new Rotation2d(yaw));
      if(Constants.kLogging){
        LogManager.addDouble("Vision/tag dist", dist);
        LogManager.addDoubleArray("Vision/pose", new double[]{pose.getX(), pose.getY(), yaw});
      }
      return pose;
    }
  }
}
