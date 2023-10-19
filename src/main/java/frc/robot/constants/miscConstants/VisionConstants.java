package frc.robot.constants.miscConstants;

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
import frc.robot.RobotId;

import java.util.ArrayList;
import java.util.List;

/**
 * Container class for vision constants.
 */
public class VisionConstants {

    /**
     * If vision is enabled
     */
    public static final boolean ENABLED = true;


    //TODO: Change these to whatever the actual distances are
    public static double kStdDevCommandEndTime = 5;

    public static ArrayList<Pair<String, Transform3d>> kCameras = new ArrayList<>();

    /**
     * Update the camera list based on the robot id detected from rio preferences.
     */
    public static void update(RobotId robotID) {
        if (robotID == RobotId.SwerveTest) {
            kCameras = new ArrayList<Pair<String, Transform3d>>(List.of(
                    new Pair<String, Transform3d>(
                            "Camera_2",
                            new Transform3d(
                                    new Translation3d(Units.inchesToMeters(15), Units.inchesToMeters(-3.3125), Units.inchesToMeters(10)),
                                    new Rotation3d(0, 0, 0)
                            )
                    )
                                                                       ));
        } else {
            kCameras = new ArrayList<Pair<String, Transform3d>>(List.of(
                    new Pair<String, Transform3d>(
                            "Left_Camera",
                            new Transform3d(
                                    new Translation3d(Units.inchesToMeters(8.996), Units.inchesToMeters(6.48), Units.inchesToMeters(37.44)),
                                    new Rotation3d(0, Units.degreesToRadians(18), 0)
                            ))
                    ,
                    new Pair<String, Transform3d>(
                            "Right_Camera",
                            new Transform3d(
                                    new Translation3d(Units.inchesToMeters(-0.75), Units.inchesToMeters(-7.125), Units.inchesToMeters(21)),
                                    new Rotation3d(0, 0, Math.PI)
                            )
                    )
                                                                       ));
        }
    }

    /**
     * Poses that use targets with an ambiguity above this amount will be ignored
     */
    public static final double highestAmbiguity = 0.02;

    // TODO: check/tune vision weight
    // How much to trust vision measurements normally

    public static final Matrix<N3, N1> kBaseVisionPoseStdDevs = new MatBuilder<>(Nat.N3(), Nat.N1()).fill(
            0.9, // x in meters (default=0.9)
            0.9, // y in meters (default=0.9)
            1000  // heading in radians. The gyroscope is very accurate, so as long as it is reset correctly, it is unnecessary to correct it with vision
                                                                                                         );
    // Increasing this makes pose estimation trust vision measurements less than distance from Apriltags increases
    // This is how much is added to std dev for vision when the closest visible Apriltag is 1 meter away

    public static final double kVisionPoseStdDevFactor = 0;

    private VisionConstants() {
    }
}