package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.constants.swerve.ModuleConstants;
import frc.robot.util.ConversionUtils;
import lib.CTREModuleState;

/**
 * Swerve module for drivetrain to be used inside of simulation.
 */
public class ModuleSim extends SubsystemBase {

    private double currentSteerPositionRad = 0;
    private double currentDrivePositionMeters = 0;
    private double currentSpeed = 0;

    private SwerveModuleState desiredState;


    protected boolean stateDeadband = true;

    public ModuleSim(ModuleConstants ignored) {
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
        desiredState = CTREModuleState.optimize(desiredState, new Rotation2d(currentSteerPositionRad));

        currentSpeed = desiredState.speedMetersPerSecond;
        currentSteerPositionRad = desiredState.angle.getRadians();
    }

    public void resetToAbsolute() {
        // does nothing when robot does not have a swerve drivetrain
    }

    public SwerveModuleState getDesiredState() {
        return desiredState;
      }

    public double getDesiredVelocity() {
        return getDesiredState().speedMetersPerSecond;
      }
    
      public Rotation2d getDesiredAngle() {
        return getDesiredState().angle;
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

    public WPI_TalonFX getDriveMotor(){
        return null;
    }

    

}