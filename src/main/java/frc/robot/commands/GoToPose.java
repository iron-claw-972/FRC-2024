package frc.robot.commands;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
  private Supplier<Rotation2d> endHeadingSupplier;
  private double maxSpeed;
  private double maxAccel;

  /**
   * Uses PathPlanner to go to a pose
   * @param poseSupplier The supplier for the pose to use
   * @param drive The drivetrain
   */
  public GoToPose(Supplier<Pose2d> poseSupplier, Supplier<Rotation2d> endHeadingSupplier, Drivetrain drive) {
    this(poseSupplier, endHeadingSupplier, AutoConstants.MAX_AUTO_SPEED, AutoConstants.MAX_AUTO_ACCEL, drive);
  }
  public GoToPose(Supplier<Pose2d> poseSupplier, Drivetrain drive) {
    this(poseSupplier, null, drive);
  }
  public GoToPose(Pose2d pose, Rotation2d endHeading, Drivetrain drive){
    this(()->pose, ()->endHeading, drive);
  }
  public GoToPose(Pose2d pose, Drivetrain drive){
    this(()->pose, null, drive);
  }

  /**
   * Uses PathPlanner to go to a pose
   * @param poseSupplier The supplier for the pose to use
   * @param endHeadingSupplier The supplier for the heading at the end of the path
   * @param maxSpeed The maximum speed to use
   * @param maxAccel The maximum acceleration to use
   * @param drive The drivetrain
   */
  public GoToPose(Supplier<Pose2d> poseSupplier, Supplier<Rotation2d> endHeadingSupplier, double maxSpeed, double maxAccel, Drivetrain drive) {
    this.poseSupplier = poseSupplier;
    this.endHeadingSupplier = endHeadingSupplier;
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
    ChassisSpeeds drivSpeeds = drive.getChassisSpeeds();
    Pose2d pose1 = new Pose2d(
      drive.getPose().getTranslation(),
      new Rotation2d(Math.atan2(drivSpeeds.vyMetersPerSecond, drivSpeeds.vxMetersPerSecond))
    );

    // Get the desired pose
    Pose2d endPose = poseSupplier.get();
    Rotation2d endRotation = endPose.getRotation();
    Rotation2d endHeading = endHeadingSupplier==null?endPose.minus(pose1).getTranslation().getAngle():endHeadingSupplier.get();
    Pose2d pose2 = new Pose2d(
      endPose.getTranslation(),
      endHeading
    );

    // Creates the command using the two points
    command = new PathPlannerCommand(List.of(pose1, pose2), drive, endRotation, maxSpeed, maxAccel);

    // get the distance to the pose.
    double dist = pose1.minus(pose2).getTranslation().getNorm();

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
