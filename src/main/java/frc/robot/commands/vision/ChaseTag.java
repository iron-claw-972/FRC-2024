package frc.robot.commands.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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
    m_rotationPID = new PIDController(1, 0, 0);
    //TODO: Make those pid constants. 
  }
  
  @Override
  public void execute() {
    //the setpoint is 2 degrees. 
    double pidValue = m_rotationPID.calculate(m_vision.getHorizontalOffsetDegrees(),2); 
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

