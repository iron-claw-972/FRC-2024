package frc.robot.commands.vision;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.DetectedObject;

/**
 * Moves toward the detected object
 * <p>Only works with the front camera
 */
public class DriveToNote extends Command {

  private double speed = DriveConstants.kMaxSpeed/2;
  private Drivetrain drive; 
  private Supplier<DetectedObject> objectSupplier;
  private DetectedObject object;
  private double angle;

  /**
   * Moves toward the detected object
   * <p>Only works with the front camera
   * @param detectedObject The supplier for the detected object to use
   * @param drive The drivetrain
   */
  public DriveToNote(Supplier<DetectedObject> detectedObject, Drivetrain drive) {
    this.objectSupplier = detectedObject;
    this.drive = drive;

    addRequirements(drive);
  }

  /**
   * Gets the object and finds the angle to it
   */
  @Override
  public void initialize(){
    object = objectSupplier.get();
  }

  /**
   * Drives toward the note
   */
  @Override
  public void execute() {
    if(object == null){
      drive.stop();
      return;
    }

    angle = object.getAngle();

    drive.driveHeading(speed*Math.cos(angle), speed*Math.sin(angle), angle, true);
  }

  /**
   * If the command is finished
   * @return True only if the object is null
   */
  @Override
  public boolean isFinished() { 
    return object == null;
  }

  /**
   * Stops the drivetrain
   * @param interrupted If the command is interrupted
   */
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}