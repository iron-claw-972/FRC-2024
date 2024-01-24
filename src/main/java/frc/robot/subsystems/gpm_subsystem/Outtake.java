package frc.robot.subsystems.gpm_subsystem;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.constants.Constants;
import frc.robot.util.ConversionUtils;
import lib.ctre_shims.TalonEncoder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;


public class Outtake extends SubsystemBase {


    private final WPI_TalonFX m_motor;
    private final TalonEncoder m_encoder;
    private final SimpleMotorFeedforward feedforward;


    private double m_velocity;
    private boolean m_enabled = false;


    public Outtake(WPI_TalonFX motor) {
        m_motor = motor;
        m_encoder = new TalonEncoder(motor);
        feedforward = new SimpleMotorFeedforward(Constants.SHOOTER_KS, Constants.SHOOTER_KV, Constants.SHOOTER_KA);
        
    }


    /**
     * Execute periodic actions for the Outtake subsystem.
     */
    public void periodic() {
        double desiredVelocity = calculateDesiredVelocity();
        double motorVelocity = ConversionUtils.MPSToFalcon(desiredVelocity, /* FLYWHEEL CIRCUMFERENCE */, /* GEAR RATIO */);


        setMotorVelocity(motorVelocity);
    }


    /**
     * Set the velocity of the outtake motor.
     *
     * @param v The desired velocity in rotations per minute.
     */
    public void setVelocity(double v) {
        m_velocity = v;
        setMotorVelocity(ConversionUtils.MPSToFalcon(m_velocity, /* FLYWHEEL CIRCUMFERENCE */, /* GEAR RATIO */));
    }
    
    /**
     * Set motor velocity with feedforward.
     *
     * @param velocity The motor velocity in control units.
     */
    private void setMotorVelocity(double velocity) {
        m_motor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(velocity));
    }


    /**
     * Get the current velocity of the outtake motor.
     *
     * @return The outtake motor velocity in rotations per minute.
     */
    public double getVelocity() {
        return Units.radiansPerSecondToRotationsPerMinute(m_encoder.getRate());
    }


    /**
     * Stop the outtake motor.
     */
    public void stop() {
        setMotorVelocity(0.0);
    }


    /**
     * Enable the outtake. This allows the outtake to start.
     *
     * @return True if the outtake is enabled.
     */
    public void enable() {
        m_enabled = true;
    }


    /**
     * Disable the outtake. This stops the outtake and prevents further action.
     *
     * @return True if the outtake is disabled.
     */
    public void disable() {
        m_enabled = false;
        stop(); // Stop the motor when disabling
    }


    /**
     * Check if the outtake is currently enabled.
     *
     * @return True if the outtake is enabled, false otherwise.
     */
    public boolean isEnabled() {
        return m_enabled;
    }


    /**
     * Calculate the desired velocity for the outtake.
     *
     * @return The calculated desired velocity.
     */
    private double calculateDesiredVelocity() {
        // Implement calculation for desired velocity
        // Might need information like launch angle, height, distance, etc.
        return 0.0; // Replace with calculation
    }
    // this should be in the command!

}
