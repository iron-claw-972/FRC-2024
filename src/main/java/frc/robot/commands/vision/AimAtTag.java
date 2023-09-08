package frc.robot.commands.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.miscConstants.FieldConstants;
import frc.robot.subsystems.Drivetrain;

public class AimAtTag extends CommandBase {
  Drivetrain m_swerve;
  PIDController m_pid;

  public AimAtTag(Drivetrain swerve){
    m_swerve = swerve; 
    m_pid = new PIDController(0, 0, 0); 
    m_pid.setTolerance(Units.degreesToRadians(1));
    addRequirements(swerve);
  }

  @Override
  public void initialize(){
    double dist = Double.POSITIVE_INFINITY;
    Translation2d closest = new Translation2d();
    Translation2d driveTranslation = m_swerve.getPose().getTranslation();
    for(AprilTag tag : FieldConstants.kAprilTags){
      Translation2d translation = tag.pose.toPose2d().getTranslation();
      double dist2 = driveTranslation.getDistance(translation);
      if(dist2 < dist){
        dist = dist2;
        closest = translation;
      }
    }
    m_pid.setSetpoint(Math.atan2(closest.getY() - driveTranslation.getY(), closest.getX() - driveTranslation.getX()));
  }
  
  @Override
  public void execute() {
    double speed = m_pid.calculate(m_swerve.getPose().getRotation().getRadians());
    m_swerve.drive(0, 0, speed, true, false);
  }

  @Override
  public void end(boolean interrupted) {
    m_swerve.stop();
  }

  @Override
  public boolean isFinished() {
    return m_pid.atSetpoint();
  }
}

