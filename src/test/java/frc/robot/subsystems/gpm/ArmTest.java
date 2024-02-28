package frc.robot.subsystems.gpm;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.constants.ArmConstants;
import frc.robot.constants.Constants;

public class ArmTest {

    /**
     * Make sure that the arm position parameters make sense.
     * <p>
     * The sanity check may catch a problem that could damage the robot.
     */
    @Test
    public void sanityTest() {
        // min angle better be less than max
        assertTrue(ArmConstants.MIN_ANGLE_RADS < ArmConstants.MAX_ANGLE_RADS);

        // range should be small
        assertTrue(-Math.PI / 4 < ArmConstants.MIN_ANGLE_RADS);
        assertTrue(ArmConstants.MAX_ANGLE_RADS < Math.PI/2);
        assertTrue(ArmConstants.MAX_ANGLE_RADS - ArmConstants.MIN_ANGLE_RADS < Math.PI / 2);

        // start angle better be reasonable
        assertTrue(ArmConstants.MIN_ANGLE_RADS <= ArmConstants.START_ANGLE_RADS);
        assertTrue(ArmConstants.START_ANGLE_RADS <= ArmConstants.MAX_ANGLE_RADS);

        // the offset should be in the range 0 to 1 (less the range of movement)
        assertTrue(0.0 <= Arm.OFFSET);
        assertTrue(Arm.OFFSET <= 1.0 - (ArmConstants.MAX_ANGLE_RADS - ArmConstants.MIN_ANGLE_RADS)/(2.0 * Math.PI));

        // the tests below actually caught a bad value...
        assertTrue(rangeCheck(ArmConstants.intakeSetpoint));
        assertTrue(rangeCheck(ArmConstants.stowedSetpoint));
        assertTrue(rangeCheck(ArmConstants.subwooferSetpoint));
        assertTrue(rangeCheck(ArmConstants.preClimbSetpoint));
        assertTrue(rangeCheck(ArmConstants.climbSetpoint));
        assertTrue(rangeCheck(ArmConstants.ampSetpoint));
    }

    /**
     * Check that an angle is within the min-max range.
     * @param setpoint arm setpoint in radians
     * @return true if the value is acceptable.
     */
    private static boolean rangeCheck(double setpoint) {
        return ArmConstants.MIN_ANGLE_RADS <= setpoint && setpoint <= ArmConstants.MAX_ANGLE_RADS;
    }
    
    /**
     * Make sure the gearing is what Kaushik says it is...
     */
    @Test
    public void gearRatioTest() {
        assertEquals(168.0, Arm.GEARING, 0.001);
    }

    /**
     * Check the motor model.
     * <p>
     * Check values that can be used for gravity compensation.
     */
    @Test
    public void motorModelTest() {
        int nMotors = ArmConstants.MOTOR_IDS.length;

        // guess the effective mass, force, and length
        double mass = 10.0;
        double force = mass * Constants.GRAVITY_ACCELERATION;
        double armLength = 0.5;
        /** maximum arm torque */
        double torque = armLength * force;

        // the motors see the geared down version of that torque
        double torqueMotors = torque / Arm.GEARING;

        // Kraken StallTorque is 7.09 Nm
        assertEquals(7.09, Arm.motorModel.stallTorqueNewtonMeters / nMotors, 0.01);
        // Kraken StallCurrent is 366.0 A
        assertEquals(366.0, Arm.motorModel.stallCurrentAmps / nMotors, 0.1);
        // Kraken torque constant (number of motors does not matter: torque increase but current also increases)
        double KtNMPerAmp = 7.09 / 366.0;
        assertEquals(KtNMPerAmp, Arm.motorModel.KtNMPerAmp, 0.0001);

        // the current the motors will need to hold up the arm
        double ampMotors = torqueMotors / Arm.motorModel.KtNMPerAmp;

        // print out the current per motor
        // System.out.println(ampMotors/nMotors);

        // so this is the estimated holding current.
        // simulation is about 2 amperes more
        assertEquals(3.8, ampMotors/nMotors, 0.1);
    }
}
