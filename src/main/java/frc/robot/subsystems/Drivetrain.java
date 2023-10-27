package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This class should include methods that will be overridden by {@link DrivetrainImpl} to be used on an actual robot
 * with a swerve style drivetrain. This class will be used when the current robot does not have a swerve style
 * drivetrain, as defined in {@link frc.robot.RobotId}.
 */
@SubsystemImpl(DrivetrainImpl.class)
public class Drivetrain extends SubsystemBase {

}
