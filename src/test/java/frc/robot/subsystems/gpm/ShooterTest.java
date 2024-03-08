package frc.robot.subsystems.gpm;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.util.Units;

public class ShooterTest {
    @BeforeEach
    public void prepare() {
    }

    @AfterEach
    public void cleanup() {
    }

    @Test
    public void motorValueTest() {
        // the free speed of a NEO Vortex is 6784.0 rpm
        assertEquals(6784.0, Units.radiansPerSecondToRotationsPerMinute(Shooter.gearbox.freeSpeedRadPerSec), 1.0);
    }

    /**
     * Check that the conversion routines are inverses of each other.
     */
    // @Test
    // public void conversionTest() {
    //     // a test rpm
    //     double rpmInput = 1500.0;
    //     // converted to a velocity
    //     double velocity = Shooter.shooterRPMToSpeed(rpmInput);
        
    //     // converting velocity back to an rpm should match the input
    //     assertEquals(rpmInput, Shooter.shooterSpeedToRPM(velocity), 0.0001);

    //     // 6000 rpm is 100 rps. circumference is pi * 4 inches
    //     assertEquals(
    //         100.0 * Math.PI * Units.inchesToMeters(4.0),
    //         Shooter.shooterRPMToSpeed(6000.0),
    //         0.0001);
    // }
}
