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

public class VisionConstants {
  /**
   * If the vision is enabled on the robot
   */
  public static final boolean kEnabled = true;

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

  // The sitance the robot should be from the grid to score
  public static final double kGridDistance = 1;

  // The limelight pose
  public static final ArrayList<Pair<String, Transform3d>> kCameras = new ArrayList<Pair<String, Transform3d>>(List.of(
    new Pair<String, Transform3d>("Camera1",
    new Transform3d(new Translation3d(0.5, 0, 0.1), new Rotation3d(0, 0, 0)))
  ));
}