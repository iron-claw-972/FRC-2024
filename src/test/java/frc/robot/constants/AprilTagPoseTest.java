package frc.robot.constants;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.ArrayList;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.constants.miscConstants.FieldConstants;
import frc.robot.util.Vision;

/**
 * Tests if all of the April tags are in the right spot
 */
public class AprilTagPoseTest {
  /**
   * Tests if all of the April tags are in the right spot
   */
  @Test
  public void testTagPoses() {
    Vision vision = new Vision(new ArrayList<Pair<String, Transform3d>>());
    for(int i = 0; i < vision.getAprilTagFieldLayout().getTags().size(); i++){
      // get the poses
      Pose3d p1 = FieldConstants.APRIL_TAGS.get(i).pose;
      Pose3d p2 = vision.getTagPose(i+1);

      // make sure the points match
      assertEquals(p1.getX(), p2.getX(), 0.0001);
      assertEquals(p1.getY(), p2.getY(), 0.0001);
      assertEquals(p1.getZ(), p2.getZ(), 0.0001);

      // make sure the rotations match
      assertEquals(p1.getRotation().getX(), p2.getRotation().getX(), 0.0001);
      assertEquals(p1.getRotation().getY(), p2.getRotation().getY(), 0.0001);
      assertEquals(p1.getRotation().getZ(), p2.getRotation().getZ(), 0.0001);
    }
  }
}
