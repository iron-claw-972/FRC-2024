package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.commands.Shoot;
import frc.robot.constants.miscConstants.VisionConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.gpm.Arm;
import frc.robot.subsystems.PowerPanel;
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
// @Disabled
public class SWMTest {
    // TODO: the math tests should not require subsystems....
    // the subsystems
    //   if these are static, then the tests run faster (but the .close() calls should be removed)
    private PowerPanel powerPanel = new PowerPanel();
    private Shooter shooter = new Shooter();
    private Arm arm = new Arm(powerPanel);
    private Drivetrain drive = new Drivetrain(null);
    private StorageIndex indexer = new StorageIndex();

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
    private enum TestCase {
        A(12.922, 5.537, 0, 0, 0),
        B(2 ,1.5 ,1.619, 0, 0),
        C(1.5 ,2 ,1.619, 0, 0),
        D(1 ,3 ,1.619, 0, 0),
        
        E(2 ,1.5 ,1.619, 3, 0) /*,
        F(-2 ,1.5 ,1.619, 3, 0),
        G(2 ,-1.5 ,1.619, 3, 0),
        H(-2 ,-1.5 ,1.619, 3, 0),
        I(1.5 ,2 ,1.619, 3, 0),
        J(-1.5 ,2 ,1.619, 3, 0),
        K(1.5 ,-2 ,1.619, 3, 0),

        L(-1.5 ,-2 ,1.7, 3, 0),
        M(2 ,1.5 ,1.7, 0, 3),
        N(-2 ,1.5 ,1.7, 0, 3),
        O(2 ,-1.5 ,1.7, 0, 3),
        P(-2 ,-1.5 ,1.7, 0, 3),
        Q(1.5 ,2 ,1.7, 0, 3),
        R(-1.5 ,2 ,1.7, 0, 3),
        S(1.5 ,-2 ,1.7, 0, 3),
        T(-1.5 ,-2 ,1.7, 0, 3),
        U(2 ,1.5 ,1.7, 3, 2),

        V(-2 ,1.5 ,1.7, 3, 2),
        W(2 ,-1.5 ,1.7, 3, 2),
        X(-2 ,-1.5 ,1.7, 3, 2),
        Y(1.5 ,2 ,1.7, 3, 2),
        Z(-1.5 ,2 ,1.7, 3, 2),
        a(1.5 ,-2 ,1.7, 3, 2),
        b(-1.5 ,-2 ,1.7, 3, 2),

        c(.2, 3 ,1.7, 1, 1) /* */
        ;
        double x, y, z, vx, vy;

        // constructor for the test cases
        TestCase(double x, double y, double z, double vx, double vy) {
            this.x = x;
            this.y = y;
            // TODO: the z value for a robot pose should be 0
            this.z = z;
            this.vx = vx;
            this.vy = vy;
        }
    };

    @AfterEach
    public void cleanup() {
        // close the subsystems
        shooter.close();
        arm.close();
        indexer.close();
        drive.close();
        powerPanel.close();
    }

    /**
     * Test if the shoot-while-moving command works.
     */
    @ParameterizedTest
    @EnumSource(TestCase.class)
    public void test_SWM(TestCase testCase) {
        // TODO: should test both the Blue and Red Alliance

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

        // set the robot position to (x, y) (with zero heading)
        drive.resetOdometry(new Pose2d(
            testCase.x,
            testCase.y,
            new Rotation2d()));

        // that should position the robot
        assertEquals(testCase.x, drive.getPose().getX(), 0.0001);
        assertEquals(testCase.y, drive.getPose().getY(), 0.0001);

        // set the robot velocities vx, vy and the robot heading
        drive.driveHeading(testCase.vx, testCase.vy, 0, true);
        // System.err.println("vx expected " + testCase.vx);
        // System.err.println("vx actual : " + drive.getChassisSpeeds().vxMetersPerSecond);
        // make sure the speeds were set
        assertEquals(testCase.vx, drive.getChassisSpeeds().vxMetersPerSecond, 0.0001);
        assertEquals(testCase.vy, drive.getChassisSpeeds().vyMetersPerSecond, 0.0001);

        // do the Shoot command calculation

        // initialize will set the speakerPose
        sh.initialize();
        // but we want to force it to be the Blue speaker for now
        sh.speakerPose = VisionConstants.BLUE_SPEAKER_POSE;
        assertTrue(VisionConstants.BLUE_SPEAKER_POSE == sh.speakerPose);

        // Execute the Command...
        sh.execute();

        // execute() should set reasonable values for horiz_angle, vert_angle, and exit_vel
        // System.err.printf("execute() returns: va %f, ha %f, and exit_vel %f\n", sh.vert_angle, sh.horiz_angle, sh.exit_vel);
        assertFalse(Double.isNaN(sh.vert_angle));
        assertFalse(Double.isNaN(sh.horiz_angle));
        assertFalse(Double.isNaN(sh.exit_vel));

        // now check the displacement
        Pose3d speakerPose = VisionConstants.BLUE_SPEAKER_POSE;
        // shooterHeight and shooterOffset have an additional offset because the shooter is offset from the arm, right?
        Rotation2d driveYaw = drive.getYaw();
        // TODO: the robot yaw is always zero!
        assertEquals(0.0, drive.getYaw().getRadians(), 0.001);

        // calculate the rotated offset.
        double offsetX = Shoot.shooterOffset * driveYaw.getCos();
        double offsetY = Shoot.shooterOffset * driveYaw.getSin();

        // the displacement should be calculated from the robot pose
        // System.err.println("displacement");
        // System.err.println(sh.displacement);
        // System.err.println(drive.getPose().getX() + offsetX - speakerPose.getX());
        // System.err.println(drive.getPose().getY() + offsetY - speakerPose.getY());
        assertEquals(drive.getPose().getX() + offsetX - speakerPose.getX(), sh.displacement.getX(), 0.0001);
        assertEquals(drive.getPose().getY() + offsetY - speakerPose.getY(), sh.displacement.getY(), 0.0001);

        // the bad news from above is the displacement is about 16 meters -- essentially the whole field length!
        // fixed by going to the Blue Alliance side of the field

        // get the instantaneous vz
        double vz = sh.exit_vel*Math.sin(sh.vert_angle);
        assertFalse(Double.isNaN(vz));

        double tx= Math.abs(sh.displacement.getX()/(sh.exit_vel*Math.cos(sh.vert_angle)*Math.cos(sh.horiz_angle)+sh.v_rx));
        double ty = Math.abs(sh.displacement.getY()/(sh.exit_vel*Math.cos(sh.vert_angle)*Math.sin(sh.horiz_angle)+sh.v_ry));
        double tz = (vz-Math.sqrt(vz*vz+19.6*sh.displacement.getZ()))/9.8;

        if (tx-tz > .01) tz = (vz+Math.sqrt(vz*vz+19.6*sh.displacement.getZ()))/9.8;

        assertFalse(Double.isNaN(tx));
        assertFalse(Double.isNaN(ty));
        assertFalse(Double.isNaN(tz));
        // System.err.println("ts: " + tx + ", " + ty + ", " + tz + ", vz" + vz);

        // test the values are close
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
        assertEquals(1.000 - 0.000, disp.getX(), 0.00001);
        assertEquals(4.000 - 0.000, disp.getY(), 0.00001);
        assertEquals(0.400 - 2.055, disp.getZ(), 0.00001);
    }
}
