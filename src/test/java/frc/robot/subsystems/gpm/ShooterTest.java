package frc.robot.subsystems.gpm;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.util.Units;

public class ShooterTest {
    Shooter shooter = new Shooter();

    @BeforeEach
    public void prepare() {
    }

    @AfterEach
    public void cleanup() {
        // shut down the motors to allow more tests.
        shooter.close();
    }

    @Test
    public void motorValueTest() {
        // the free speed of a NEO Vortex is 6784.0 rpm
        assertEquals(6784.0, Units.radiansPerSecondToRotationsPerMinute(Shooter.gearbox.freeSpeedRadPerSec), 1.0);
    }

    /**
     * Check that the conversion routines are inverses of each other.
     */
    @Test
    public void conversionTest() {
        // a test rpm
        double rpmInput = 1500.0;
        // converted to a velocity
        double velocity = Shooter.shooterRPMToSpeed(rpmInput);
        
        // converting velocity back to an rpm should match the input
        assertEquals(rpmInput, Shooter.shooterSpeedToRPM(velocity), 0.0001);

        // 6000 rpm is 100 rps. circumference is pi * 4 inches. Geared up by 2.0.
        assertEquals(
            (1.0 / Shooter.gearRatio) * 100.0 * Math.PI * Units.inchesToMeters(4.0),
            Shooter.shooterRPMToSpeed(6000.0),
            0.0001);
    }

    @Test
    public void spinUpTest() {
        // shooter RPM should be at zero
        assertEquals(0.0, shooter.getRightMotorRPM(), 0.0001);
        assertEquals(0.0, shooter.getLeftMotorRPM(), 0.00001);

        // set the target speeds to zero
        shooter.setTargetRPM(0.0, 0.0);

        // shooter PID should be satisfied
        // satisfied is cached? Do some calculations to fill the cache.
        shooter.leftPID.calculate(0.0);
        shooter.rightPID.calculate(0.0);
        assertTrue(shooter.atSetpoint());

        // set the motor speed to 1000 RPM
        shooter.setTargetRPM(1000.0, 1000.0);
        // we should not be satisfied
        assertFalse(shooter.atSetpoint());

        // run some simulations
        // TODO: after 3 seconds, we are still 75 RPM away!
        // Tolerance is 80 RPM, so we are at the set point.
        for (int i = 0; i < 150; i++) {
            shooter.simulationPeriodic();
            shooter.periodic();
            System.out.println(shooter.getLeftMotorRPM());
        }

        // we should have settled down
        assertTrue(shooter.atSetpoint());
    }

    // We have shooter data at
    // https://docs.google.com/spreadsheets/d/10JpBlUdVOniuSvXAG5M9LxhE3K0s10cQR82EBcGcMh8/edit#gid=0
    // but it needs interpretation
    // I think
    // A is the measurement in unknown units (bananas?)
    // B is the set RPM of the measurement
    // C = A1*(385.826772/(2*18))^(1/2)
    // D = ((((C1+C2+C3)/3)/(4*PI()))*60)/2 ... is this the effective RPM?
    //     this averages Ci
}
