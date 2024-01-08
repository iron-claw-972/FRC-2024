package frc.robot.constants.miscConstants;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

public class FieldConstants {
  public static final double kFieldLength = Units.inchesToMeters(54*12 + 3.25); // meters
  public static final double kFieldWidth = Units.inchesToMeters(26*12 + 3.5); // meters

  // Array to use if it can't find the April tag field layout
  // TODO: Add these
  public static final ArrayList<AprilTag> APRIL_TAGS = new ArrayList<AprilTag>(List.of(
    new AprilTag(1, new Pose3d(Units.inchesToMeters(610.77), Units.inchesToMeters( 42.19), Units.inchesToMeters(18.22), new Rotation3d(0.0, 0.0, Math.PI))),
    new AprilTag(2, new Pose3d(Units.inchesToMeters(610.77), Units.inchesToMeters(108.19), Units.inchesToMeters(18.22), new Rotation3d(0.0, 0.0, Math.PI))),
    new AprilTag(3, new Pose3d(Units.inchesToMeters(610.77), Units.inchesToMeters(174.19), Units.inchesToMeters(18.22), new Rotation3d(0.0, 0.0, Math.PI))),
    new AprilTag(4, new Pose3d(Units.inchesToMeters(636.96), Units.inchesToMeters(265.74), Units.inchesToMeters(27.38), new Rotation3d(0.0, 0.0, Math.PI))),
    new AprilTag(5, new Pose3d(Units.inchesToMeters( 14.25), Units.inchesToMeters(265.74), Units.inchesToMeters(27.38), new Rotation3d(0.0, 0.0, 0.0))),
    new AprilTag(6, new Pose3d(Units.inchesToMeters( 40.45), Units.inchesToMeters(174.19), Units.inchesToMeters(18.22), new Rotation3d(0.0, 0.0, 0.0))),
    new AprilTag(7, new Pose3d(Units.inchesToMeters( 40.45), Units.inchesToMeters(108.19), Units.inchesToMeters(18.22), new Rotation3d(0.0, 0.0, 0.0))),
    new AprilTag(8, new Pose3d(Units.inchesToMeters( 40.45), Units.inchesToMeters( 42.19), Units.inchesToMeters(18.22), new Rotation3d(0.0, 0.0, 0.0))),
    new AprilTag(9, new Pose3d()),
    new AprilTag(10, new Pose3d()),
    new AprilTag(11, new Pose3d()),
    new AprilTag(12, new Pose3d()),
    new AprilTag(13, new Pose3d()),
    new AprilTag(14, new Pose3d()),
    new AprilTag(15, new Pose3d()),
    new AprilTag(16, new Pose3d())
  ));
}
