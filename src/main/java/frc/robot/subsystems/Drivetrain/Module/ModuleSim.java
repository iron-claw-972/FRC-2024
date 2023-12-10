package frc.robot.subsystems.drivetrain.module;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.constants.globalConst;
import frc.robot.constants.swerve.ModuleConstants;

/**
 * Swerve module for drivetrain to be used inside of simulation.
 *
 * @see frc.robot.subsystems.drivetrain.module.Module
 */
public class ModuleSim extends Module {

    private double currentSteerPositionRad = 0;
    private double currentDrivePositionMeters = 0;
    private double currentSpeed = 0;

    public ModuleSim(ModuleConstants moduleConstants, ShuffleboardTab swerveTab) {
        super(moduleConstants, swerveTab);
    }

    /**
     * Returns the current state of the module.
     *
     * @return the current state of the module
     */
    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(currentSpeed, new Rotation2d(currentSteerPositionRad));
    }

    /**
     * Updates the simulation.
     */
    @Override
    public void periodic() {
        currentDrivePositionMeters += currentSpeed * globalConst.LOOP_TIME;
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     * @param isOpenLoop   whether to use closed/open loop control for drive velocity
     */
    @Override
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

    /**
     * Gets the simulated angle of the module.
     */
    @Override
    public Rotation2d getAngle() {
        return new Rotation2d(currentSteerPositionRad);
    }

    @Override
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                currentDrivePositionMeters,
                new Rotation2d(currentSteerPositionRad)
        );
    }

    @Override
    public SwerveModuleState getDesiredState() {
        return getState();
    }
}