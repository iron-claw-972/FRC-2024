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
import frc.robot.constants.swerve.DriveConstants;

public class VisionConstants {
  /**
   * If April tag vision is enabled on the robot
   */
  public static final boolean ENABLED = true;

  // If odometry should be updated using vision during auto
  public static final boolean ENABLED_AUTO = false;

  // If odometry should be updated using vision while running the GoToPose and GoToPosePID commands in teleop
  public static final boolean ENABLED_GO_TO_POSE = false;

  // If vision should use manual calculations
  public static final boolean USE_MANUAL_CALCULATIONS = true;

  // The number to multiply the distance to the April tag by
  // Only affects manual calculations
  // To find this, set it to 1 and measure the actual distance and the calculated distance
  public static final double DISTANCE_SCALE = 0.9;

  /*
   * The standard deviations to use for the vision
   */
  public static final Matrix<N3, N1> VISION_STD_DEVS = new MatBuilder<>(Nat.N3(), Nat.N1()).fill(
    0.00443, // x in meters (default=0.9)
    0.00630, // y in meters (default=0.9)
    1000  // heading in radians. The gyroscope is very accurate, so as long as it is reset correctly it is unnecessary to correct it with vision
  );

  // The highest ambiguity to use. Ambiguities higher than this will be ignored.
  // Only affects calculations using PhotonVision, not manual calculations
  public static final double HIGHEST_AMBIGUITY = 0.02;

  // The distance the robot should be from the grid to score in inches
  public static final double GRID_DISTANCE = Units.metersToInches(DriveConstants.kRobotWidthWithBumpers)/2+1;

  // Distance from the April tag to the single substation in the x direction
  public static final double SINGLE_SUBSTATION_DISTANCE = Units.inchesToMeters(79.395);

  // X in m of the single substations
  public static final double BLUE_SINGLE_SUBSTATION_X = FieldConstants.APRIL_TAGS.get(3).pose.getX() - SINGLE_SUBSTATION_DISTANCE;
  public static final double RED_SINGLE_SUBSTATION_X = FieldConstants.APRIL_TAGS.get(4).pose.getX() + SINGLE_SUBSTATION_DISTANCE;

  // How far the robot should be from the shelf when aligning
  public static final double SHELF_DISTANCE = 1;

  /** The Blue Shelf X position to align to */
  public static final double BLUE_SHELF_X = FieldConstants.APRIL_TAGS.get(3).pose.getX() - SHELF_DISTANCE;
  /** The Red Shelf X position to align to */
  public static final double RED_SHELF_X = FieldConstants.APRIL_TAGS.get(4).pose.getX() + SHELF_DISTANCE;

  /** The Y position of the top double substation shelf, the one closer to the wall */
  // 15 from april tag to edge of portal (where cones are accessible). That area is 34.21 inches wide, just aim for the center
  public static final double TOP_SHELF_Y = FieldConstants.APRIL_TAGS.get(3).pose.getY() + Units.inchesToMeters(15 + (34.21 / 2));
  /** The Y position of the bottom double substation shelf, the one closer to the grids */
  // 15 from april tag to edge of portal (where cones are accessible). That area is 34.21 inches wide, just aim for the center
  public static final double BOTTOM_SHELF_Y = FieldConstants.APRIL_TAGS.get(3).pose.getY() - Units.inchesToMeters(15 + (34.21 / 2));

  // The camera poses
  public static final ArrayList<Pair<String, Transform3d>> CAMERAS = new ArrayList<Pair<String, Transform3d>>(List.of(
    new Pair<String, Transform3d>(
      "Camera1",
      new Transform3d(
        new Translation3d(Units.inchesToMeters(8.996), Units.inchesToMeters(6.48), Units.inchesToMeters(37.44)),
        new Rotation3d(0, Units.degreesToRadians(18), 0)
      ))
    ,
    new Pair<String, Transform3d>(
      "Camera2",
      new Transform3d(
        new Translation3d(Units.inchesToMeters(-0.75), Units.inchesToMeters(-7.125), Units.inchesToMeters(21)),
        new Rotation3d(0, 0, Math.PI)
      )
    )
  ));
}