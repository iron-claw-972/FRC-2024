package frc.robot.commands.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.miscConstants.AutoConstants;
import frc.robot.subsystems.Drivetrain.SwerveDrive.DrivetrainImpl;
import frc.robot.util.ConversionUtils;
import frc.robot.util.PathGroupLoader;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;


public class PathPlannerCommand extends SequentialCommandGroup {

    public PathPlannerCommand(ArrayList<PathPoint> waypoints, DrivetrainImpl drive) {
        this(waypoints, drive, true);
    }

    public PathPlannerCommand(ArrayList<PathPoint> waypoints, DrivetrainImpl drive, boolean useAllianceColor) {
        this(new ArrayList<PathPlannerTrajectory>(List.of(PathPlanner.generatePath(
                new PathConstraints(AutoConstants.MAX_AUTO_SPEED, AutoConstants.MAX_AUTO_ACCEL),
                waypoints.get(0),
                waypoints.get(1)
                                                                                  ))), 0, drive, false, useAllianceColor, true);
    }

    public PathPlannerCommand(ArrayList<PathPoint> waypoints, DrivetrainImpl drive, boolean useAllianceColor, double maxSpeed, double maxAccel) {
        this(new ArrayList<PathPlannerTrajectory>(List.of(PathPlanner.generatePath(
                new PathConstraints(maxSpeed, maxAccel),
                waypoints.get(0),
                waypoints.get(1)
                                                                                  ))), 0, drive, false, useAllianceColor, true);
    }

    public PathPlannerCommand(String pathGroupName, int pathIndex, DrivetrainImpl drive) {
        this(PathGroupLoader.getPathGroup(pathGroupName), pathIndex, drive, true, true, false);
    }

    public PathPlannerCommand(String pathGroupName, int pathIndex, DrivetrainImpl drive, boolean resetPose) {
        this(PathGroupLoader.getPathGroup(pathGroupName), pathIndex, drive, resetPose, true, false);
    }

    public PathPlannerCommand(List<PathPlannerTrajectory> pathGroup, int pathIndex, DrivetrainImpl drive, boolean resetPose) {
        this(pathGroup, pathIndex, drive, resetPose, true, false);
    }

    public PathPlannerCommand(List<PathPlannerTrajectory> pathGroup, int pathIndex, DrivetrainImpl drive, boolean resetPose, boolean useAllianceColor, boolean isPerpetual) {

        addRequirements(drive);
        if (pathIndex < 0 || pathIndex > pathGroup.size() - 1) {
            throw new IndexOutOfBoundsException("Path index out of range");
        }

        addCommands(
                new InstantCommand(() -> {
                    PathPlannerTrajectory path = PathPlannerTrajectory.transformTrajectoryForAlliance(
                            pathGroup.get(pathIndex), DriverStation.getAlliance());
                    if (resetPose) {
                        drive.resetOdometry(ConversionUtils.absolutePoseToPathPlannerPose(path.getInitialHolonomicPose(), DriverStation.getAlliance()));
                    }
                }),
                createSwerveControllerCommand(
                        pathGroup.get(pathIndex),
                        useAllianceColor ? // Pose supplier
                                () -> ConversionUtils.absolutePoseToPathPlannerPose(drive.getPose(), DriverStation.getAlliance()) :
                                                                                                                                          drive::getPose,
                        drive.getPathplannerXController(), // X controller can't normal PID as pathplanner has Feed Forward
                        drive.getPathplannerYController(), // Y controller can't normal PID as pathplanner has Feed Forward
                        drive.getPathplannerRotationController(), // Rotation controller can't normal PID as pathplanner has Feed Forward
                        (chassisSpeeds) -> {
                            drive.setChassisSpeeds(chassisSpeeds, false);
                        }, // chassis Speeds consumer
                        useAllianceColor,  // use Alliance color
                        drive, // Requires this drive subsystem
                        isPerpetual
                                             )
                   );
    }

    public static PPSwerveControllerCommand createSwerveControllerCommand(
            PathPlannerTrajectory trajectory, Supplier<Pose2d> poseSupplier,
            PIDController xController, PIDController yController, PIDController rotationController,
            Consumer<ChassisSpeeds> outputChassisSpeeds, boolean useAllianceColor, DrivetrainImpl drive, boolean isPerpetual) {
        if (isPerpetual) {
            return new PPSwerveControllerCommandPerpetual(
                    trajectory,
                    poseSupplier,
                    xController,
                    yController,
                    rotationController,
                    outputChassisSpeeds,
                    useAllianceColor,
                    drive
            );
        }

        return new PPSwerveControllerCommand(
                trajectory,
                poseSupplier,
                xController,
                yController,
                rotationController,
                outputChassisSpeeds,
                useAllianceColor,
                drive
        );
    }
}
