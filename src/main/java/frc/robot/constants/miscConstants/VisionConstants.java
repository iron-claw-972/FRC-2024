package frc.robot.constants.miscConstants;
/**
 * Container class for vision constants.
 */

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
  /**
   * If the vision is enabled on the robot
   */
  public static final boolean kEnabled = true;

  public static final boolean kUseManualCalculations = false;

  /*
   * The standard deviations to use for the vision
   */
  public static final Matrix<N3, N1> kVisionStdDevs = new MatBuilder<>(Nat.N3(), Nat.N1()).fill(
    0.01, // x in meters (default=0.9)
    0.016, // y in meters (default=0.9)
    1000  // heading in radians. The gyroscope is very accurate, so as long as it is reset correctly it is unnecessary to correct it with vision
  );

  // The highest ambiguity to use
  public static final double kHighestAmbiguity = 1;

  // The distance the robot should be from the grid to score in inches
  public static final double kGridDistance = 15;

  // Distance from the April tag to the single substation in the x direction
  public static final double kSingleSubstationDistance = Units.inchesToMeters(79.395);

  // X in m of the single substations
  public static final double kBlueSingleSubstationX = FieldConstants.kAprilTags.get(3).pose.getX() - kSingleSubstationDistance;
  public static final double kRedSingleSubstationX = FieldConstants.kAprilTags.get(4).pose.getX() + kSingleSubstationDistance;

  // How far the robot should be from the shelf when aligning
  public static final double kShelfDistance = 1;

  /** The Blue Shelf X position to align to */
  public static final double kBlueShelfX = FieldConstants.kAprilTags.get(3).pose.getX() - kShelfDistance;
  /** The Red Shelf X position to align to */
  public static final double kRedShelfX = FieldConstants.kAprilTags.get(4).pose.getX() + kShelfDistance;

  /** The Y position of the top double substation shelf, the one closer to the wall */
  // 15 from april tag to edge of portal (where cones are accessible). That area is 34.21 inches wide, just aim for the center
  public static double kTopShelfY = FieldConstants.kAprilTags.get(3).pose.getY() + Units.inchesToMeters(15 + (34.21 / 2));
  /** The Y position of the bottom double substation shelf, the one closer to the grids */
  // 15 from april tag to edge of portal (where cones are accessible). That area is 34.21 inches wide, just aim for the center
  public static double kBottomShelfY = FieldConstants.kAprilTags.get(3).pose.getY() - Units.inchesToMeters(15 + (34.21 / 2));

  // The camera poses
  public static final ArrayList<Pair<String, Transform3d>> kCameras = new ArrayList<Pair<String, Transform3d>>(List.of(
    new Pair<String, Transform3d>(
      "Camera1",
      new Transform3d(
        new Translation3d(Units.inchesToMeters(8.996), Units.inchesToMeters(6.48), Units.inchesToMeters(37.44)),
        new Rotation3d(0, Units.degreesToRadians(18), 0)
      ))
    // ,
    // new Pair<String, Transform3d>(
    //   "Camera2",
    //   new Transform3d(
    //     new Translation3d(Units.inchesToMeters(-0.75), Units.inchesToMeters(-7.125), Units.inchesToMeters(21)),
    //     new Rotation3d(0, 0, Math.PI)
    //   )
    // )
  ));
}