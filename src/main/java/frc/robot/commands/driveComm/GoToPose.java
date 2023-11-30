package frc.robot.commands.driveComm;

import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DoNothing;
import frc.robot.commands.SupplierCommand;
import frc.robot.commands.autoComm.PathPlannerCommand;
import frc.robot.constants.AutoConstants;
import frc.robot.subsystems.Drivetrain.swerveDrive.swerveDriveImpl;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

/**
 * Moves the robot to a pose using PathPlanner
 */
public class GoToPose extends SequentialCommandGroup {

    private final swerveDriveImpl drive;
    private final Supplier<Pose2d> poseSupplier;
    private final double maxSpeed;
    private final double maxAccel;

    /**
     * Uses PathPlanner to go to a pose
     *
     * @param poseSupplier The supplier for the pose to use
     * @param drive        The drivetrain
     */
    public GoToPose(Supplier<Pose2d> poseSupplier, swerveDriveImpl drive) {
        this(poseSupplier, AutoConstants.MAX_AUTO_SPEED, AutoConstants.MAX_AUTO_ACCEL, drive);
    }

    /**
     * Uses PathPlanner to go to a pose
     *
     * @param poseSupplier The supplier for the pose to use
     * @param maxSpeed     The maximum speed to use
     * @param maxAccel     The maximum acceleration to use
     * @param drive        The drivetrain
     */
    public GoToPose(Supplier<Pose2d> poseSupplier, double maxSpeed, double maxAccel, swerveDriveImpl drive) {
        this.poseSupplier = poseSupplier;
        this.maxSpeed = maxSpeed;
        this.maxAccel = maxAccel;
        this.drive = drive;
        addCommands(
                new SupplierCommand(this::createCommand, drive)
                   );
    }

    /**
     * Creates the PathPlanner command and schedules it
     */
    public Command createCommand() {
        Command command;
        // Gets the current position of the robot for the start of the path
        PathPoint point1 = PathPoint.fromCurrentHolonomicState(
                drive.getPose(),
                drive.getChassisSpeeds()

                // set the control lengths. This controls how strong the heading is
                // aka how much the robot will curve to get to the point.
                // We want it to follow a straight line, and with swerve, it isn't too necessary.
                                                              ).withControlLengths(0.001, 0.001);

        // get the desired score pose
        Pose2d pose = poseSupplier.get();

        // Uses the pose to find the end point for the path
        PathPoint point2 = new PathPoint(
                pose.getTranslation(),
                pose.getRotation(),
                pose.getRotation(),
                0
                // set the control lengths. This controls how strong the heading is
                // aka how much the robot will curve to get to the point.
                // We want it to follow a straight line, and with swerve, it isn't too necessary.
        ).withControlLengths(0.001, 0.001);

        // Creates the command using the two points
        command = new PathPlannerCommand(
                new ArrayList<PathPoint>(List.of(point1, point2)),
                drive,
                false,
                maxSpeed,
                maxAccel
        );

        // get the distance to the pose.
        double dist = drive.getPose().minus(pose).getTranslation().getNorm();

        // if greater than 4m or less than 20 cm, don't run it. If the path is too small pathplanner makes weird paths.
        if (dist > 4) {
            command = new DoNothing();
            DriverStation.reportWarning("Alignment Path too long, doing nothing, GoToPose.java", false);
        } else if (dist < 0.2) {
            command = new DoNothing();
            DriverStation.reportWarning("Alignment Path too short, doing nothing, GoToPose.java", false);
        }

        return command;
    }
}
