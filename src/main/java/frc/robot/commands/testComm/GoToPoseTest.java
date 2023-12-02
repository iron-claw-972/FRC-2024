package frc.robot.commands.testComm;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.TestConstants;
import frc.robot.subsystems.drivetrain.swerveDrive.swerveDriveImpl;

/**
 * Tests the odometry of the robot by driving a certain distance and calculating the error.
 */
public class GoToPoseTest extends CommandBase {

    private final swerveDriveImpl drive;

    private double startTime;
    private Pose2d finalPose;
    private Pose2d error;

    public GoToPoseTest(swerveDriveImpl drive) {
        this.drive = drive;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
        finalPose = new Pose2d(
                drive.getRequestedXPos(drive.getPose().getX()), drive.getRequestedYPos(drive.getPose().getY()),
                new Rotation2d(drive.getRequestedHeading(drive.getYaw().getRadians()))
        );

    }

    @Override
    public void execute() {
        drive.runChassisPID(finalPose.getX(), finalPose.getY(), finalPose.getRotation().getRadians());
    }

    @Override
    public boolean isFinished() {
        // TODO: the current PID values don't allow the command to finish
        double errorMarginMeters = TestConstants.TRANSLATION_ERROR;
        double errorMarginRadians = Units.degreesToRadians(10);
        error = drive.getPose().relativeTo(finalPose);
        // if robot thinks its precision is < 0.1 to the target we inputted, it will stop, so then we can see how off it is
        return Math.abs(error.getX()) < errorMarginMeters && Math.abs(error.getY()) < errorMarginMeters && Math.abs(error.getRotation().getRadians()) < errorMarginRadians;
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
        System.out.println(Timer.getFPGATimestamp() - startTime);
        System.out.println(error.getX());
        System.out.println(error.getY());
        System.out.println(error.getRotation().getRadians());
    }
}