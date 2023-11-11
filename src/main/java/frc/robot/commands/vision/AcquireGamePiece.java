package frc.robot.commands.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.constants.miscConstants.TestConstants;
import frc.robot.constants.miscConstants.VisionConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Vision;

/**
 * Tests the odometry of the robot by driving a certain distance and calculating the error.
 */
public class AcquireGamePiece extends CommandBase {

  private Drivetrain m_drive; 
  private double x_offset_rads; 
  private double distance; 
  private double rotation_pid_output; 
  private double distance_pid_output; 


  private PIDController m_rotation_controller; 
  private PIDController m_distance_controller; 

  private Vision m_vision = new Vision();  
  
  public AcquireGamePiece(Drivetrain drive) {
    m_drive = drive;

    m_rotation_controller = new PIDController(VisionConstants.kRotationP, VisionConstants.kRotationI, VisionConstants.kRotationD);
    m_rotation_controller.setTolerance(1); //1 degree of tolerance

    m_distance_controller = new PIDController(VisionConstants.kDistanceP, VisionConstants.kDistanceI, VisionConstants.kDistanceD); 
    m_distance_controller.setTolerance(5); //5 cm of tolerance

    addRequirements(drive);
  }

  @Override
  public void execute() {
    x_offset_rads = m_vision.getHorizontalOffset()*(Math.PI/180);
    distance = m_vision.getDistance();
     
    rotation_pid_output = m_rotation_controller.calculate(x_offset_rads); 
    distance_pid_output = m_distance_controller.calculate(distance); 

    //TODO: this may need to go in xSpeed, not ySpeed
    m_drive.driveHeading(0, distance_pid_output, rotation_pid_output, false);
    
    
  }

  @Override
  public boolean isFinished() { 
    if(m_rotation_controller.atSetpoint() && m_distance_controller.atSetpoint()){
      return true; 
    }

    //otherwise, return false 
    return false; 

  }

  @Override
  public void end(boolean interrupted) {

  }
}