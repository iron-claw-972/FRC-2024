package frc.robot.commands.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Vision;

/**
 * Moves toward the detected object
 * <p>Only works with the front camera
 */
public class AcquireGamePiecePID extends CommandBase {

  private Drivetrain m_drive; 
  private Vision m_vision;

  private PIDController m_rotationPID; 
  private PIDController m_distancePID; 
  
/**
 * Moves toward the detected object
 * <p>Only works with the front camera
   * @param drive The drivetrain
   * @param vision The vision
   */
  public AcquireGamePiecePID(Drivetrain drive, Vision vision) {
    m_drive = drive;
    m_vision = vision;

    // Use similar PIDs to the drivetrain
    m_rotationPID = new PIDController(DriveConstants.kHeadingP, 0, DriveConstants.kHeadingD);
    m_rotationPID.setTolerance(Units.degreesToRadians(3)); //3 degree of tolerance
    m_rotationPID.setSetpoint(0);

    m_distancePID = new PIDController(DriveConstants.kTranslationalP, 0, DriveConstants.kTranslationalD); 
    m_distancePID.setTolerance(0.02); //2 cm of tolerance
    m_distancePID.setSetpoint(0);

    addRequirements(drive);
  }

  /**
   * Resets PIDs
   */
  @Override
  public void initialize(){
    m_rotationPID.reset();
    m_distancePID.reset();
  }

  /**
   * Gets the x offset and distance and drives toward the object
   */
  @Override
  public void execute() {
    if(m_vision.getHorizontalOffset().length==0){
      m_drive.stop();
      return;
    }
    //get horizontal offset from cam to center of game piece + distance from cam to game piece from networktables
    double xOffset = -Units.degreesToRadians(m_vision.getHorizontalOffset()[0]);
    double distance = m_vision.getDistance()[0];
    
    //power to send to the rotation parameter of the drive command
    double rotationOutput = m_rotationPID.calculate(xOffset);
    rotationOutput = MathUtil.clamp(rotationOutput,-DriveConstants.kMaxAngularSpeed, DriveConstants.kMaxAngularSpeed); 
    
    //calculate the output speed we need to move the robot to the target
    double distanceOutput = -m_distancePID.calculate(distance, 0); 
    distanceOutput = MathUtil.clamp(distanceOutput, -DriveConstants.kMaxSpeed, DriveConstants.kMaxSpeed); 
    
    double xSpeed = distanceOutput*Math.cos(xOffset);
    double ySpeed = distanceOutput*Math.sin(xOffset);

    m_drive.drive(xSpeed, ySpeed, rotationOutput, false, false);
  }

  /**
   * If the command is finished
   * @return If both PIDs are at the setpoints
   */
  @Override
  public boolean isFinished() { 
    return false;
    // return m_rotationPID.atSetpoint() && m_distancePID.atSetpoint();
  }

  /**
   * Stops the drivetrain
   * @param interrupted If the command is interrupted
   */
  @Override
  public void end(boolean interrupted) {
    m_drive.stop();
  }
}