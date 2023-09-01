package frc.robot.commands.test;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

/**
 * Drives robot in a circle. There is often drift due to inaccuracy.
 */
public class CircleDrive extends CommandBase {

    private final Drivetrain drive;
    private double steerPosition = 0, prevTime;

    public CircleDrive(Drivetrain drive) {
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        prevTime = WPIUtilJNI.now() * 1e-6;
        steerPosition = 0;
    }

    @Override
    public void execute() {
        double currentTime = WPIUtilJNI.now() * 1e-6;
        steerPosition = MathUtil.angleModulus(steerPosition + (currentTime - prevTime) * drive.getRequestedSteerVelocity(0));
        drive.setModuleStates(new SwerveModuleState[]{
                new SwerveModuleState(drive.getRequestedDriveVelocity(0), new Rotation2d(steerPosition)),
                new SwerveModuleState(drive.getRequestedDriveVelocity(0), new Rotation2d(steerPosition)),
                new SwerveModuleState(drive.getRequestedDriveVelocity(0), new Rotation2d(steerPosition)),
                new SwerveModuleState(drive.getRequestedDriveVelocity(0), new Rotation2d(steerPosition))
        }, false);
        prevTime = currentTime;
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}
