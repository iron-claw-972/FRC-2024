package frc.robot.constants.miscConstants;
/**
 * Container class for vision constants.
 */

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
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
    0.9, // x in meters (default=0.9)
    0.9, // y in meters (default=0.9)
    1000  // heading in radians. The gyroscope is very accurate, so as long as it is reset correctly it is unnecessary to correct it with vision
  );
}