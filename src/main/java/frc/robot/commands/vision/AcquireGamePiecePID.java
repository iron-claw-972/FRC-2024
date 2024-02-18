package frc.robot.commands.vision;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.DetectedObject;

/**
 * Moves toward the detected object
 * <p>Only works with the front camera
 */
public class AcquireGamePiecePID extends Command {

  private Drivetrain drive; 
  private Supplier<DetectedObject> objectSupplier;

  /**
   * Moves toward the detected object
   * <p>Only works with the front camera
   * @param detectedObject The supplier for the detected object to use
   * @param drive The drivetrain
   */
  public AcquireGamePiecePID(Supplier<DetectedObject> detectedObject, Drivetrain drive) {
    this.objectSupplier = detectedObject;
    this.drive = drive;

    addRequirements(drive);
  }

  /**
   * Does nothing
   */
  @Override
  public void initialize(){
  }

  /**
   * Gets the x offset and distance and drives toward the object
   */
  @Override
  public void execute() {
    DetectedObject object = objectSupplier.get();
    if(object == null){
      drive.stop();
      return;
    }

    drive.driveWithPID(object.pose.getX(), object.pose.getY(), object.getAngle());
  }

  /**
   * If the command is finished
   * @return Always false
   */
  @Override
  public boolean isFinished() { 
    return false;
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