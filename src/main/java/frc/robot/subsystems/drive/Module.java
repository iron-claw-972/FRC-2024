package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.constants.swerve.ModuleConstants;
import frc.robot.util.ConversionUtils;

/**
 * Swerve module for drivetrain to be used inside of simulation.
 */
public class Module extends SubsystemBase {

    private double currentSteerPositionRad = 0;
    private double currentDrivePositionMeters = 0;
    private double currentSpeed = 0;

    protected boolean stateDeadband = true;

    public Module(ModuleConstants ignored) {
    }

    /**
     * Updates the simulation
     */
    @Override
    public void periodic() {
        currentDrivePositionMeters += currentSpeed * Constants.LOOP_TIME;
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     * @param isOpenLoop   whether to use closed/open loop control for drive velocity
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
            currentSpeed = 0;
            return;
        }
        // Optimize the reference state to avoid spinning further than 90 degrees
        desiredState = SwerveModuleState.optimize(desiredState, new Rotation2d(currentSteerPositionRad));

        currentSpeed = desiredState.speedMetersPerSecond;
        currentSteerPositionRad = desiredState.angle.getRadians();
    }

    public void resetToAbsolute() {
        // does nothing when robot does not have a swerve drivetrain
    }

    // TODO: Comment
    public void stop() {
        currentSpeed = 0;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                currentSpeed,
                getAngle()
        );
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                currentDrivePositionMeters,
                new Rotation2d(currentSteerPositionRad)
        );
    }

    /**
     * Gets the simulated angle of the module.
     */
    public Rotation2d getAngle() {
        return new Rotation2d(currentSteerPositionRad);
    }

    // TODO: Comment
    public void setStateDeadband(boolean enabled) {
        stateDeadband = enabled;
    }

}