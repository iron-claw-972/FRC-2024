package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.controller.PIDController;

/**
 * Some students do not trust PIDControler.atSetpoint(), so they have written their own.
 * Stop chasing ghosts.
 * <p>
 * This test class demonstrates that .atSetpoint() works.
 * <p>
 * For the PIDController class, see {@link https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/controller/PIDController.html}.
 * <p>
 * For the .atSetpoint() source code, see {@link https://github.wpilib.org/allwpilib/docs/release/java/src-html/edu/wpi/first/math/controller/PIDController.html#line.279}
 * <p>
 * .atSetpoint() relies on the class field m_positionError, which is updated in .calculate() and .setSetpoint().
 * So changing the setpoint or calculating a new PID value will be reflected in .atSetpoint() immediately.
 */
public class PIDTest {
    // make a PIDController
    static final double kp = 1.0;
    static final double ki = 0.1;
    static final double kd = 0.0;
    PIDController pid = new PIDController(kp, ki, kd);

    // error bound for floating point equality
    static final double eps = 1.0e-8;

    /**
     * Initialize the PIDController as it would be in a constructor.
     */
    @BeforeEach
    public void prepare() {
        // set the tolerance
        pid.setTolerance(0.01);
    }

    @AfterEach
    public void cleanup() {
        // reclaim the resources
        pid.close();
    }

    /**
     * Test the tolerance.
     */
    @Test
    public void toleranceTest() {
        // make sure the tolerance was set
        assertEquals(0.01, pid.getPositionTolerance(), eps);

        // while here, check the velocity tolerance is infinite
        assertTrue(Double.isInfinite(pid.getVelocityTolerance()));
    }

    /**
     * Test setpoints and measurements.
     */
    @Test
    public void SetpointTest() {
        // WPI did something sensible:
        // initially, the PIDController does not know the current setpoint nor the current measurement,
        // so it cannot be at the setpoint.
        assertFalse(pid.atSetpoint());

        // provide the controller a first setpoint of 2.0
        pid.setSetpoint(2.000);

        // although the controller has a setpoint, it still does not have a measurement.
        // it does not know if it is at the setpoint.
        assertFalse(pid.atSetpoint());

        // provide the controller a first measurement with calculate()
        pid.calculate(1.999);

        // check the position error
        assertEquals(2.000 - 1.999, pid.getPositionError(), 0.00001);

        // now the controller is at the setpoint
        assertTrue(pid.atSetpoint());

        // now the measurement or the setpoint can be changed at will

        // change the setpoint
        pid.setSetpoint(1.500);

        // that should update the position error
        assertEquals(1.500 - 1.999, pid.getPositionError(), eps);

        // and we are no longer at the setpoint...
        assertFalse(pid.atSetpoint());

        // make a new measurement
        pid.calculate(1.501);

        // check the position error was updated
        assertEquals(1.500 - 1.501, pid.getPositionError(), eps);

        // at the new setpoint
        assertTrue(pid.atSetpoint());
    }
}
