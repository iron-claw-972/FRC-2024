package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SubsystemImpl;

/**
 * This class should include methods that will be overridden by {@link DrivetrainImpl} to be used on an actual robot
 * with a swerve style drivetrain. This class will be used when the current robot does not have a swerve style
 * drivetrain, as defined in {@link frc.robot.RobotId}.
 */
@SubsystemImpl(DrivetrainImpl.class)
public class Drivetrain extends SubsystemBase {

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        speed of the robot in the x direction (forward) in m/s
     * @param ySpeed        speed of the robot in the y direction (sideways) in m/s
     * @param rot           angular rate of the robot in rad/s
     * @param fieldRelative whether the provided x and y speeds are relative to the field
     * @param isOpenLoop    whether to use velocity control for the drive motors
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean isOpenLoop) {
        // TODO: Implement what this method should do as a placeholder
    }

}
