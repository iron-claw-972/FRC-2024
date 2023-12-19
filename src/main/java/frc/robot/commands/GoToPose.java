package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto_comm.PathPlannerCommand;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.miscConstants.VisionConstants;
import frc.robot.subsystems.Drivetrain;

/**
* Moves the robot to a pose using PathPlanner
*/
public class GoToPose extends SequentialCommandGroup {

  private Drivetrain drive;
  private Supplier<Pose2d> poseSupplier;
  private double maxSpeed;
  private double maxAccel;

  /**
   * Uses PathPlanner to go to a pose
   * @param poseSupplier The supplier for the pose to use
   * @param drive The drivetrain
   */
  public GoToPose(Supplier<Pose2d> poseSupplier, Drivetrain drive) {
    this(poseSupplier, AutoConstants.MAX_AUTO_SPEED, AutoConstants.MAX_AUTO_ACCEL, drive);
  }
  public GoToPose(Pose2d pose, Drivetrain drive){
    this(()->pose, drive);
  }

  /**
   * Uses PathPlanner to go to a pose
   * @param poseSupplier The supplier for the pose to use
   * @param maxSpeed The maximum speed to use
   * @param maxAccel The maximum acceleration to use
   * @param drive The drivetrain
   */
  public GoToPose(Supplier<Pose2d> poseSupplier, double maxSpeed, double maxAccel, Drivetrain drive) {
    this.poseSupplier = poseSupplier;
    this.maxSpeed = maxSpeed;
    this.maxAccel = maxAccel;
    this.drive = drive;
    addCommands(
      new InstantCommand(()->drive.setVisionEnabled(VisionConstants.ENABLED_GO_TO_POSE)),
      new SupplierCommand(() -> createCommand(), drive),
      new InstantCommand(()->drive.setVisionEnabled(true))
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

    // if greater than 6m or less than 10 cm, don't run it. If the path is too small pathplanner makes weird paths.
    if (dist > 6) {
      command = new DoNothing();
      DriverStation.reportWarning("Alignment Path too long, doing nothing, GoToPose.java", false);
    } else if (dist < 0.1) {
      command = new DoNothing();
      DriverStation.reportWarning("Alignment Path too short, doing nothing, GoToPose.java", false);
    }

    return command.handleInterrupt(()->drive.setVisionEnabled(true));
  }
}
