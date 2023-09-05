package frc.robot.commands.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Vision;

public class AimAtTag extends CommandBase {
  Vision m_vision; 
  Drivetrain m_swerve; 
  PIDController m_controller; 
  public AimAtTag(Vision vision, Drivetrain swerve){
    m_vision = vision;
    m_swerve = swerve; 
    m_controller = new PIDController(0, 0, 0); 
  }
  
  @Override
  public void execute() { 
    m_con
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return false; 
  }


}

