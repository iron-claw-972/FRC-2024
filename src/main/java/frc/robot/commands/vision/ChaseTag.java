package frc.robot.commands.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.miscConstants.FieldConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Vision;

public class ChaseTag extends CommandBase {
  Drivetrain m_swerve;
  PIDController m_rotationPID;
  Vision m_vision; 

  private double m_pidValue;
  private double m_wheelVelocity;


  public ChaseTag(Drivetrain swerve, Vision vision){
    m_swerve = swerve;
    m_vision = vision; 
    m_rotationPID = new PIDController(1, 0, 0);
    //TODO: Make those pid constants. 
  }

  @Override
  public void initialize(){
    setDrivetrainToTurnMode();
  }
  
  @Override
  public void execute() {
    //the setpoint is 2 degrees. 
    m_pidValue = m_rotationPID.calculate(m_vision.getHorizontalOffsetDegrees(),2); 
    m_wheelVelocity = MathUtil.clamp(m_pidValue,-4.3,4.3); 
    setWheelSpeeds(m_wheelVelocity);
    
  }

  @Override
  public void end(boolean interrupted) {
    
  }

  @Override
  public boolean isFinished() {
    return false; 
  }

  private void setDrivetrainToTurnMode(){
    //set drivetrain to "turn mode"
    m_swerve.setModuleStates(new SwerveModuleState[] {
      new SwerveModuleState(0, new Rotation2d(Units.degreesToRadians(-45))),
      new SwerveModuleState(0, new Rotation2d(Units.degreesToRadians(45))),
      new SwerveModuleState(0, new Rotation2d(Units.degreesToRadians(45))),
      new SwerveModuleState(0, new Rotation2d(Units.degreesToRadians(-45)))
    });
  }

  private void setWheelSpeeds(double velocity){
    //set drivetrain wheel velocities. 

    //may need to invert these velocities. 
    m_swerve.setModuleStates(new SwerveModuleState[] {
      new SwerveModuleState(velocity, new Rotation2d(Units.degreesToRadians(-45))),
      new SwerveModuleState(velocity, new Rotation2d(Units.degreesToRadians(45))),
      new SwerveModuleState(velocity, new Rotation2d(Units.degreesToRadians(45))),
      new SwerveModuleState(velocity, new Rotation2d(Units.degreesToRadians(-45)))
    });
  }
  
}

