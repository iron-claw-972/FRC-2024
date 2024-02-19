package frc.robot.util;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.RepeatedTest;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.gpm.Shooter;
import frc.robot.subsystems.gpm.Arm;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class SWMTest {
    public static Shoot sh = new Shoot(new Shooter(), new Arm(), new Drivetrain(null));
    public static int idx = 0;
    public double[][] test_cases = {
       // x, y, z ,vx, vy
        {2 ,1.5 ,1.7, 0, 0},
        {1.5 ,2 ,1.7, 0, 0},
        {1 ,3 ,1.7, 0, 0},
        {2 ,1.5 ,1.7, 3, 0},
        {-2 ,1.5 ,1.7, 3, 0},//
        {2 ,-1.5 ,1.7, 3, 0},
        {-2 ,-1.5 ,1.7, 3, 0},
        {1.5 ,2 ,1.7, 3, 0},
        {-1.5 ,2 ,1.7, 3, 0},
        {1.5 ,-2 ,1.7, 3, 0},
        {-1.5 ,-2 ,1.7, 3, 0},
    };
    public final int NUM = 11; // num test cases

    @AfterEach
    public void cleanup() {
    }
    public void go(int idx) {
        sh.displacement = new Pose3d(
            test_cases[idx][0],
            test_cases[idx][1],
            -test_cases[idx][2],
            new Rotation3d());
        // set v_rx
        sh.v_rx = test_cases[idx][3];
        // set v_ry
        sh.v_ry = test_cases[idx][4];

        sh.execute();
        double vz = sh.exit_vel*Math.sin(sh.vert_angle);
        double tx= Math.abs(sh.displacement.getX()/(sh.exit_vel*Math.cos(sh.vert_angle)*Math.cos(sh.horiz_angle)+sh.v_rx)),
        ty = Math.abs(sh.displacement.getY()/(sh.exit_vel*Math.cos(sh.vert_angle)*Math.sin(sh.horiz_angle)+sh.v_ry)),
        // tx, ty could switch sin(horiz) and cos(horiz) depending if horiz is relative to the vertical or horizontal axis (x-cos, y-sin is horizontal).
        tz = (vz-Math.sqrt(vz*vz+19.6*sh.displacement.getZ()))/9.8;
        // NOTE:+ instead of -
        System.err.println(tx+", " + ty + ", " + tz+"vz"+vz);
        assertEquals(tx, ty, 0.001);
        assertEquals(tx, tz, 0.001);

    }
    /**
     * Test if shoot while moving command works.
     */
    @RepeatedTest(NUM)
    public void test_SWM() {
        go(idx++);
    }
    @Test
    public void testo() {
        // 0,0 is the bottom right corner of the blue wall (m)
        Pose3d disp = new Pose3d(1,4,.4,new Rotation3d()).relativeTo(new Pose3d(0,0,2.055,new Rotation3d()));
        System.out.println(disp.getX()+","+disp.getY()+","+disp.getZ()+"done");
    }
}
