package frc.robot.commands.auto;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindHolonomic;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPoint;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.miscConstants.AutoConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.ConversionUtils;
import frc.robot.util.PathGroupLoader;


public class PathPlannerCommand extends SequentialCommandGroup {
  
  public PathPlannerCommand(ArrayList<PathPoint> waypoints, Drivetrain drive) {
    this(waypoints, drive, true);
  }

  public PathPlannerCommand(ArrayList<PathPoint> waypoints, Drivetrain drive, boolean useAllianceColor) {
    this(new ArrayList<PathPlannerTrajectory>(Arrays.asList(new PathPlannerTrajectory(
      new PathConstraints(AutoConstants.kMaxAutoSpeed, AutoConstants.kMaxAutoAccel),
      waypoints.get(0),
      waypoints.get(1)
    ))), 0, drive, false, useAllianceColor, true);
  }

  public PathPlannerCommand(ArrayList<PathPoint> waypoints, Drivetrain drive, boolean useAllianceColor, double maxSpeed, double maxAccel) {
    this(new ArrayList<PathPlannerTrajectory>(Arrays.asList(PathPlanner.generatePath(
      new PathConstraints(maxSpeed, maxAccel),
      waypoints.get(0),
      waypoints.get(1)
    ))), 0, drive, false, useAllianceColor, true);
  }
  
  public PathPlannerCommand(String pathGroupName, int pathIndex, Drivetrain drive) {
    this(PathGroupLoader.getPathGroup(pathGroupName), pathIndex, drive, true, true, false); 
  }
  
  public PathPlannerCommand(String pathGroupName, int pathIndex, Drivetrain drive, boolean resetPose) {
    this(PathGroupLoader.getPathGroup(pathGroupName), pathIndex, drive, resetPose, true, false); 
  }

  public PathPlannerCommand(List<PathPlannerTrajectory> pathGroup, int pathIndex, Drivetrain drive, boolean resetPose){
    this(pathGroup, pathIndex, drive, resetPose, true, false);
  }

  public PathPlannerCommand(List<PathPlannerTrajectory> pathGroup, int pathIndex, Drivetrain drive, boolean resetPose, boolean useAllianceColor, boolean isPerpetual) {

    addRequirements(drive);
    if (pathIndex < 0 || pathIndex > pathGroup.size() - 1) {
      throw new IndexOutOfBoundsException("Path index out of range"); 
    }
    
    addCommands(
      new InstantCommand( () -> {
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
          () -> drive.getPose(), 
        drive.getPathplannerXController(), // X controller can't normal PID as pathplanner has Feed Forward 
        drive.getPathplannerYController(), // Y controller can't normal PID as pathplanner has Feed Forward 
        drive.getPathplannerRotationController(), // Rotation controller can't normal PID as pathplanner has Feed Forward 
        (chassisSpeeds) -> { drive.setChassisSpeeds(chassisSpeeds, false); }, // chassis Speeds consumer
        useAllianceColor,  // use Alliance color
        drive, // Requires this drive subsystem
        isPerpetual
      )
    );
  }
  
  public static PathfindHolonomic createSwerveControllerCommand(
      PathPlannerTrajectory trajectory, Supplier<Pose2d> poseSupplier, 
      PIDController xController, PIDController yController, PIDController rotationController, 
      Consumer<ChassisSpeeds> outputChassisSpeeds, boolean useAllianceColor, Drivetrain drive, boolean isPerpetual) {
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

    return new PathfindHolonomic(
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
