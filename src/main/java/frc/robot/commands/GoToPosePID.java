
package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.miscConstants.VisionConstants;
import frc.robot.subsystems.Drivetrain;

/**
 * Runs the chassis PIDs to move the robot to a specific pose. 
 */
public class GoToPosePID extends CommandBase {

  private Drivetrain m_drive; 
  
  private Supplier<Pose2d> m_poseSupplier;
  private Pose2d m_pose;
  
  /**
   * Runs the chassis PIDs to move the robot to a specific pose. 
   */
  public GoToPosePID(Supplier<Pose2d> pose, Drivetrain drive) {
    m_drive = drive; 
    m_poseSupplier = pose;
    
    addRequirements(drive);
  }
    
  @Override
  public void initialize() {
    m_pose = m_poseSupplier.get();
    m_drive.enableVision(VisionConstants.ENABLED_GO_TO_POSE);
  }

  @Override
  public void execute() {
    m_drive.runChassisPID(m_pose.getX(), m_pose.getY(), m_pose.getRotation().getRadians()); 
  }

  @Override
  public boolean isFinished() {
    // TODO: the current PID values don't allow the command to finish
    return m_drive.getXController().atSetpoint() && m_drive.getYController().atSetpoint() && m_drive.getRotationController().atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.stop();
    m_drive.enableVision(true);
  }
}