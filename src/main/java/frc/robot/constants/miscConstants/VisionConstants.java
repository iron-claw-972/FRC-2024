package frc.robot.constants.miscConstants;
/**
 * Container class for vision constants.
 */

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.swerve.DriveConstants;

public class VisionConstants {
  /**
   * If April tag vision is enabled on the robot
   */
  public static final boolean ENABLED = true;

  // If odometry should be updated using vision during auto
  public static final boolean ENABLED_AUTO = true;

  // If odometry should be updated using vision while running the GoToPose and GoToPosePID commands in teleop
  public static final boolean ENABLED_GO_TO_POSE = true;

  // If vision should use manual calculations
  public static final boolean USE_MANUAL_CALCULATIONS = false;

  // The number to multiply the distance to the April tag by
  // Only affects manual calculations
  // To find this, set it to 1 and measure the actual distance and the calculated distance
  public static final double DISTANCE_SCALE = 1;
  public static final double X_OFFSET_SCALE = 139.28/94.24;
  public static final double Y_OFFSET_SCALE = 114/60.84;

  /*
   * The standard deviations to use for the vision
   */
  public static final Matrix<N3, N1> VISION_STD_DEVS = MatBuilder.fill(Nat.N3(), Nat.N1(),
    0.00443, // x in meters (default=0.9)
    0.00630, // y in meters (default=0.9)
    1000  // heading in radians. The gyroscope is very accurate, so as long as it is reset correctly it is unnecessary to correct it with vision
  );

  // The highest ambiguity to use. Ambiguities higher than this will be ignored.
  // Only affects calculations using PhotonVision, not manual calculations
  public static final double HIGHEST_AMBIGUITY = 0.02;

  // Speaker poses
  public static final Pose3d BLUE_SPEAKER_POSE = new Pose3d(
    FieldConstants.APRIL_TAGS.get(6).pose.getX() + Units.inchesToMeters(9),
    FieldConstants.APRIL_TAGS.get(6).pose.getY(),
    Units.inchesToMeters(80.5),
    new Rotation3d(0, 0, 0)
  );
  public static final Pose3d RED_SPEAKER_POSE = new Pose3d(
    FieldConstants.APRIL_TAGS.get(3).pose.getX() - Units.inchesToMeters(9),
    BLUE_SPEAKER_POSE.getY(),
    BLUE_SPEAKER_POSE.getZ(),
    BLUE_SPEAKER_POSE.getRotation().rotateBy(new Rotation3d(0, 0, Math.PI))
  );

  // The amp poses to align to
  public static final Pose2d BLUE_AMP_POSE = new Pose2d(
    FieldConstants.APRIL_TAGS.get(5).pose.getX(),
    FieldConstants.APRIL_TAGS.get(5).pose.getY() - DriveConstants.kRobotWidthWithBumpers/2,
    new Rotation2d(-Math.PI/2)
  );
  public static final Pose2d RED_AMP_POSE = new Pose2d(
    FieldConstants.APRIL_TAGS.get(4).pose.getX(),
    BLUE_AMP_POSE.getY(),
    BLUE_AMP_POSE.getRotation()
  );

  // How close we have to get to the amp before scoring in it (meters and radians)
  public static final double AMP_TOLERANCE_DISTANCE = 0.3;
  public static final double AMP_TOLERANCE_ANGLE = Units.degreesToRadians(15);

  // The podium poses to align to
  public static final Pose2d BLUE_PODIUM_POSE = new Pose2d(
    FieldConstants.APRIL_TAGS.get(13).pose.getX() - Units.inchesToMeters(82.75) - DriveConstants.kRobotWidthWithBumpers/2,
    FieldConstants.APRIL_TAGS.get(13).pose.getY(),
    new Rotation2d(0)
  );
  public static final Pose2d RED_PODIUM_POSE = new Pose2d(
    FieldConstants.APRIL_TAGS.get(12).pose.getX() + Units.inchesToMeters(82.75) + DriveConstants.kRobotWidthWithBumpers/2,
    BLUE_PODIUM_POSE.getY(),
    new Rotation2d(Math.PI).minus(BLUE_PODIUM_POSE.getRotation())
  );

  public enum CHAIN_POSES{
    RED_LEFT(FieldConstants.APRIL_TAGS.get(10)),
    RED_RIGHT(FieldConstants.APRIL_TAGS.get(11)),
    RED_CENTER(FieldConstants.APRIL_TAGS.get(12)),
    BLUE_CENTER(FieldConstants.APRIL_TAGS.get(13)),
    BLUE_LEFT(FieldConstants.APRIL_TAGS.get(14)),
    BLUE_RIGHT(FieldConstants.APRIL_TAGS.get(15));

    private double dist1 = Units.inchesToMeters(50);
    private double dist2 = Units.inchesToMeters(9);
    public final Pose2d pose1;
    public final Pose2d pose2;
    private CHAIN_POSES(AprilTag tag){
      pose1 = tag.pose.toPose2d().plus(new Transform2d(new Translation2d(dist1, 0/*tag.pose.toPose2d().getRotation()*/), new Rotation2d(Math.PI)));
      pose2 = tag.pose.toPose2d().plus(new Transform2d(new Translation2d(dist2, 0/*tag.pose.toPose2d().getRotation()*/), new Rotation2d(Math.PI)));
    }
  }

  // The camera poses
  // TODO: Add these
  public static final ArrayList<Pair<String, Transform3d>> APRIL_TAG_CAMERAS = new ArrayList<Pair<String, Transform3d>>(List.of(
    new Pair<String, Transform3d>(
      "Camera1",
      new Transform3d(
        new Translation3d(Units.inchesToMeters(-12.55), Units.inchesToMeters(-9.705), Units.inchesToMeters(9.5)),
        new Rotation3d(0, Units.degreesToRadians(-50), Math.PI)
      )),
    new Pair<String, Transform3d>(
      "Camera2",
      new Transform3d(
        new Translation3d(Units.inchesToMeters(-0.75), Units.inchesToMeters(-7.125), Units.inchesToMeters(21)),
        new Rotation3d(0, 0, Math.PI)
      )
    )
  ));

  //how many frames ago to pull position from (latency calculation)
  public static final double objectLatencySeconds = 0.8;
  public static final double objectPositionTolerance = 0;
}