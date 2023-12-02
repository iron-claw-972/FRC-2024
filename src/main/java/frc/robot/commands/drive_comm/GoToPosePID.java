package frc.robot.commands.drive_comm;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.swerve.SwerveDriveImpl;

import java.util.function.Supplier;

/**
 * Runs the chassis PIDs to move the robot to a specific pose.
 */
public class GoToPosePID extends CommandBase {

    private final SwerveDriveImpl drive;

    private final Supplier<Pose2d> pose;

    /**
     * Runs the chassis PIDs to move the robot to a specific pose.
     */
    public GoToPosePID(Supplier<Pose2d> pose, SwerveDriveImpl drive) {
        this.drive = drive;
        this.pose = pose;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        drive.runChassisPID(pose.get().getX(), pose.get().getY(), pose.get().getRotation().getRadians());
    }

    @Override
    public boolean isFinished() {
        // TODO: the current PID values don't allow the command to finish
        return drive.getXController().atSetpoint() && drive.getYController().atSetpoint() && drive.getRotationController().atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}