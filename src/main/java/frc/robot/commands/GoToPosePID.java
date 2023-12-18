
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.miscConstants.VisionConstants;

/**
 * Runs the chassis PIDs to move the robot to a specific pose. 
 */
public class GoToPosePID extends CommandBase {

  private Drivetrain drive; 
  
  private Supplier<Pose2d> poseSupplier;
  private Pose2d pose;
  
  /**
   * Runs the chassis PIDs to move the robot to a specific pose. 
   * @param pose The pose supplier to go to
   * @param drive The drivetrain
   */
  public GoToPosePID(Supplier<Pose2d> pose, Drivetrain drive) {
    this.drive = drive;
    this.poseSupplier = pose;

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    pose = poseSupplier.get();
    drive.enableVision(VisionConstants.ENABLED_GO_TO_POSE);
  }

  @Override
  public void execute() {
    drive.driveWithPID(pose.getX(), pose.getY(), pose.getRotation().getRadians()); 
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.stop();
    m_drive.enableVision(true);
  }

  @Override
    public boolean isFinished() {
        // TODO: 2024, create instances of the PID controllers in this class
        // TODO: the current PID values don't allow the command to finish 2023
        return drive.getXController().atSetpoint() && drive.getYController().atSetpoint() && drive.getRotationController().atSetpoint();
    }
}