package frc.robot.commands.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Vision;

public class ChaseTag extends CommandBase {
  Drivetrain m_swerve;
  PIDController m_rotationPID;
  Vision m_vision; 

  public ChaseTag(Drivetrain swerve, Vision vision){
    m_swerve = swerve;
    m_vision = vision;
    m_rotationPID = new PIDController(
      m_swerve.getRotationController().getP(),
      m_swerve.getRotationController().getI(),
      m_swerve.getRotationController().getD()
    );
    m_rotationPID.setTolerance(Units.degreesToRadians(1));
  }
  
  @Override
  public void execute() {
    double pidValue = m_rotationPID.calculate(m_vision.getHorizontalOffsetDegrees(), 0);
    double wheelVelocity = MathUtil.clamp(pidValue,-0.25,0.25); 
    m_swerve.drive(0, 0, wheelVelocity, false, false);
  }

  @Override
  public void end(boolean interrupted) {
    m_swerve.stop();
  }

  @Override
  public boolean isFinished() {
    return m_rotationPID.atSetpoint(); 
  }
}

