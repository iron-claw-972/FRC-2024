package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.RepeatedTest;
import org.junit.jupiter.api.RepetitionInfo;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.commands.Shoot;
import frc.robot.constants.miscConstants.VisionConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.gpm.Arm;
import frc.robot.subsystems.gpm.Shooter;
import frc.robot.subsystems.gpm.StorageIndex;

/**
 * Shoot-While-Moving tests.
 * <p>
 * Currently disabled because the tests are creating NaNs.
 * <p>
 * The math tests should also not need to build the subsystems.
 * The math in the Shoot command is opaque.
 */
@Disabled
public class SWMTest {
    // the subsystems
    //   if these are static, then the tests run faster (but the .close() calls should be removed)
    public Shooter shooter = new Shooter();
    public Arm arm = new Arm();
    public Drivetrain drive = new Drivetrain(null);
    public StorageIndex indexer = new StorageIndex();

    /**
     * The Shoot command
     */
    public Shoot sh = new Shoot(shooter, arm, drive, indexer);

    /*
     * make drivetrain in the test
     * method to set pose, reset pose
     * set chassis speed or drive() to make x,y velocities.
     */
    // TODO: these have the input conditions, but they do not have the appropriate answer
    public static final double[][] test_cases = {
        // x, y, z, vx, vy
        {12.922, 5.537, 0, 0, 0},
        {2 ,1.5 ,1.619, 0, 0},
        {1.5 ,2 ,1.619, 0, 0},
        {1 ,3 ,1.619, 0, 0},

        {2 ,1.5 ,1.619, 3, 0},
        {-2 ,1.5 ,1.619, 3, 0},
        {2 ,-1.5 ,1.619, 3, 0},
        {-2 ,-1.5 ,1.619, 3, 0},
        {1.5 ,2 ,1.619, 3, 0},
        {-1.5 ,2 ,1.619, 3, 0},
        {1.5 ,-2 ,1.619, 3, 0},

        {-1.5 ,-2 ,1.7, 3, 0},
        {2 ,1.5 ,1.7, 0, 3},
        {-2 ,1.5 ,1.7, 0, 3},
        {2 ,-1.5 ,1.7, 0, 3},
        {-2 ,-1.5 ,1.7, 0, 3},
        {1.5 ,2 ,1.7, 0, 3},
        {-1.5 ,2 ,1.7, 0, 3},
        {1.5 ,-2 ,1.7, 0, 3},
        {-1.5 ,-2 ,1.7, 0, 3},
        {2 ,1.5 ,1.7, 3, 2},

        {-2 ,1.5 ,1.7, 3, 2},
        {2 ,-1.5 ,1.7, 3, 2},
        {-2 ,-1.5 ,1.7, 3, 2},
        {1.5 ,2 ,1.7, 3, 2},
        {-1.5 ,2 ,1.7, 3, 2},
        {1.5 ,-2 ,1.7, 3, 2},
        {-1.5 ,-2 ,1.7, 3, 2},

        {.2, 3 ,1.7, 1, 1},
    };
    // number of test cases (= test_case.length). The value must be a literal to work in the tests below.
    public final int NUM = 29;

    @AfterEach
    public void cleanup() {
        // close the subsystems
        shooter.close();
        arm.close();
        indexer.close();
        drive.close();
    }

    /**
     * Make sure we will run all the tests.
     */
    @Test
    public void testLength() {
        // System.out.println(test_cases.length);
        assertEquals(test_cases.length, NUM);
    }

    /**
     * Test if shoot while moving command works.
     */
    @RepeatedTest(NUM)
    public void test_SWM(RepetitionInfo ri) {
        // get the zero-based index into test cases
        int idx = ri.getCurrentRepetition() - 1;

        /* sh.displacement = new Pose3d(
            test_cases[idx][0],
            test_cases[idx][1],
            -test_cases[idx][2],
            new Rotation3d());
        // set v_rx
        sh.v_rx = test_cases[idx][3];
        // set v_ry
        sh.v_ry = test_cases[idx][4];
        */

        // set the robot position to (x, y) (with bogus heading)
        sh.drive.resetOdometry(new Pose2d(
            test_cases[idx][0],
            test_cases[idx][1],
            new Rotation2d()));

        // that should position the robot
        assertEquals(test_cases[idx][0], drive.getPose().getX(), 0.0001);
        assertEquals(test_cases[idx][1], drive.getPose().getY(), 0.0001);

        // set robot velocities vx, vy
        sh.drive.driveHeading(test_cases[idx][3], test_cases[idx][4], 0, true);

        System.err.println("vx expected " + test_cases[idx][3]);
        // this value is a NaN rather than vx
        System.err.println("vx actual : " + sh.drive.getChassisSpeeds().vxMetersPerSecond);

        // do the Shoot command calculation (no initialize())
        sh.execute();

        // execute() should set reasonable values for horiz_angle, vert_angle, and exit_vel
        System.err.printf("execute() returns: va %f, ha %f, and exit_vel %f\n", sh.vert_angle, sh.horiz_angle, sh.exit_vel);

        // now check the displacement
        Pose3d speakerPose = VisionConstants.RED_SPEAKER_POSE;
        // shooterHeight and shooterOffset have an additional offset because the shooter is offset from the arm, right?
        Rotation2d driveYaw = drive.getYaw();
        // TODO: the robot yaw is always zero!
        assertEquals(0.0, drive.getYaw().getRadians(), 0.001);

        // calculate the rotated offset.
        double jimmyX = Shoot.shooterOffset * driveYaw.getCos();
        double jimmyY = Shoot.shooterOffset * driveYaw.getSin();

        // the displacement should be calculated from the robot pose
        // System.err.println("displacement");
        // System.err.println(sh.displacement);
        // System.err.println(drive.getPose().getX() + jimmyX - speakerPose.getX());
        // System.err.println(drive.getPose().getY() + jimmyY - speakerPose.getY());
        assertEquals(drive.getPose().getX() + jimmyX - speakerPose.getX(), sh.displacement.getX(), 0.0001);
        assertEquals(drive.getPose().getY() + jimmyY - speakerPose.getY(), sh.displacement.getY(), 0.0001);

        // TODO: the bad news from above is the displacement is about 16 meters -- essentially the whole field length!

        // get the instantaneous vz
        // this is a NaN
        double vz = sh.exit_vel*Math.sin(sh.vert_angle);

        // because sh.exit_vel is a NaN
        System.err.println(sh.exit_vel);
        // assertTrue(Double.isNaN(sh.exit_vel));
        // because vert angle is a NaN
        System.err.println(sh.vert_angle);
        // assertTrue(Double.isNaN(sh.vert_angle));

        double tx= Math.abs(sh.displacement.getX()/(sh.exit_vel*Math.cos(sh.vert_angle)*Math.cos(sh.horiz_angle)+sh.v_rx)),
        ty = Math.abs(sh.displacement.getY()/(sh.exit_vel*Math.cos(sh.vert_angle)*Math.sin(sh.horiz_angle)+sh.v_ry)),
        tz = (vz-Math.sqrt(vz*vz+19.6*sh.displacement.getZ()))/9.8;

        if (tx-tz > .01) tz = (vz+Math.sqrt(vz*vz+19.6*sh.displacement.getZ()))/9.8;

        // tx, ty, tz, and vz are NaNs!
        System.err.println("ts: " + tx + ", " + ty + ", " + tz + ", vz" + vz);

        // These tests are crazy: they are comparing NaNs to NaNs
        assertEquals(tx, ty, 0.001);
        assertEquals(tx, tz, 0.001);
    }

    @Test
    public void testo() {
        // 0,0 is the bottom right corner of the blue wall (m)
        // make a displacement
        Pose3d disp = new Pose3d(1, 4, .4,new Rotation3d()).relativeTo(new Pose3d(0, 0, 2.055, new Rotation3d()));

        // System.out.println(disp.getX()+","+disp.getY()+","+disp.getZ()+" done");
        // System.out.println(disp);

        // make sure the displacement is as expected
        assertEquals(1.0 - 0.000, disp.getX(), 0.00001);
        assertEquals(4.0 - 0.000, disp.getY(), 0.00001);
        assertEquals(0.4 - 2.055, disp.getZ(), 0.00001);
    }
}
