package frc.robot.util;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
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
    public double[][] test_cases = {
       //x,y,z  ,vx,vy
        {2,2,1.7,0, 0}
    };
    @BeforeEach
    public void prepare() {
    }

    @AfterEach
    public void cleanup() {
    }
    public void go() {

        sh.execute();
        double vz = sh.exit_vel*Math.sin(sh.vert_angle);
        double tx= Math.abs(sh.displacement.getX()/(sh.exit_vel*Math.cos(sh.vert_angle)*Math.sin(sh.horiz_angle)+sh.v_rx)),
        ty = Math.abs(sh.displacement.getY()/(sh.exit_vel*Math.cos(sh.vert_angle)*Math.cos(sh.horiz_angle)+sh.v_ry)),
        tz = (-vz+Math.sqrt(vz*vz-9.8*4*sh.displacement.getZ()))/19.6;
        assertEquals(0, tx -ty, 0.01);
        assertEquals(0, tx -tz, 0.01);

    }
    @Test
    public void test_SWM() {
        for (double[] ar: test_cases) {

        sh.displacement = new Pose3d(
            ar[0],
            ar[1],
            2.1-.5,
            new Rotation3d());
        // set v_rx
        sh.v_rx = 0;
        // set v_ry
        sh.v_ry = 0;
        }
    }
    /**
     * Test if shoot while moving command works.
     */
    @Test
    public void test_SWM1() {
        // requires these setters in Shoot() to be commented out :(
        // set displacement
        sh.displacement = new Pose3d(
            2.0,
            2.0,
            2.1-.5,
            new Rotation3d());
        // set v_rx
        sh.v_rx = 0;
        // set v_ry
        sh.v_ry = 0;
        go();
    }

    @Test
    public void test_SWM2() {
        // requires these setters in Shoot() to be commented out :(
        // set displacement
        sh.displacement = new Pose3d(2.0,
        2.0,
        2.1-.5,
        new Rotation3d());
        // set v_rx
        sh.v_rx = 3;
        // set v_ry
        sh.v_ry = 0;

    go();
    }

    @Test
    public void test_SWM3() {
        // requires these setters in Shoot() to be commented out :(
        // set displacement
        sh.displacement = new Pose3d(2.0,
        2.0,
        2.1-.5,
        new Rotation3d());
        // set v_rx
        sh.v_rx = 3;
        // set v_ry
        sh.v_ry = 0;
        go();
    }
    @Test
    public void test_SWM4() {
        // requires these setters in Shoot() to be commented out :(
        // set displacement
        sh.displacement = new Pose3d(-2.0,
        -2.0,
        2.1-.5,
        new Rotation3d());
        // set v_rx
        sh.v_rx = 3;
        // set v_ry
        sh.v_ry = 0;
        go();
    }
    @Test
    public void test_SWM5() {
        // requires these setters in Shoot() to be commented out :(
        // set displacement
        sh.displacement = new Pose3d(2.0,
        -2.0,
        2.1-.5,
        new Rotation3d());
        // set v_rx
        sh.v_rx = 3;
        // set v_ry
        sh.v_ry = 0;
        go();
    }
    @Test
    public void test_SWM6() {
        // requires these setters in Shoot() to be commented out :(
        // set displacement
        sh.displacement = new Pose3d(-2.0,
        2.0,
        2.1-.5,
        new Rotation3d());
        // set v_rx
        sh.v_rx = 3;
        // set v_ry
        sh.v_ry = 0;
        go();
    }
}
