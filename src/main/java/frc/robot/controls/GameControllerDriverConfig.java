package frc.robot.controls;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.GoToPose;
import frc.robot.commands.drive_comm.SetFormationX;
import frc.robot.constants.GlobalConst;
import frc.robot.constants.miscConstants.VisionConstants;
import frc.robot.subsystems.drivetrain.swerve.SwerveDriveImpl;
import frc.robot.util.MathUtils;
import frc.robot.util.Node;
import lib.controllers.GameController;
import lib.controllers.GameController.Axis;
import lib.controllers.GameController.Button;
import lib.controllers.GameController.DPad;

/**
 * Driver controls for the generic game controller.
 */
public class GameControllerDriverConfig extends BaseDriverConfig {
  // Stores the values for the Node the driver selects
  private int[] selectionValues = new int[3];
  // The selected node
  private Node selectedNode = new Node();
  // The timestamp of when the driver last pressed a selection button
  private double selectTimestamp = 0;
  
  private final GameController kDriver = new GameController(GlobalConst.DRIVER_JOY);

  @Override
  public void configureControls() { 
    // set the wheels to X
    kDriver.get(Button.X).onTrue(new SetFormationX(super.getDrivetrain()));
    
    // Resets the modules to absolute if they are having the unresolved zeroing error
    kDriver.get(Button.A).onTrue(new InstantCommand(() -> getDrivetrain().resetModulesToAbsolute()));

    // Alignment controls
    kDriver.get(DPad.LEFT).onTrue(new InstantCommand(() -> select(1)));
    kDriver.get(DPad.UP).onTrue(new InstantCommand(() -> select(2)));
    kDriver.get(DPad.RIGHT).onTrue(new InstantCommand(() -> select(3)));
    kDriver.get(DPad.DOWN).onTrue(new InstantCommand(() -> select(0)));
    // Grid alignment
    kDriver.get(Button.RB).whileTrue(new GoToPose(() -> getSelectedPose(), getDrivetrain()));
    // Single substation x and angle alignment
    kDriver.get(Button.LB).whileTrue(new GoToPose(() -> new Pose2d(
      DriverStation.getAlliance()==Alliance.Blue?VisionConstants.BLUE_SINGLE_SUBSTATION_X:VisionConstants.RED_SINGLE_SUBSTATION_X,
      getDrivetrain().getPose().getY(),
      new Rotation2d(Math.PI/2)
    ), getDrivetrain()));
    // Double substation alignment
    kDriver.get(Button.Y).whileTrue(new GoToPose(() -> new Pose2d(
      DriverStation.getAlliance()==Alliance.Blue?VisionConstants.BLUE_SHELF_X:VisionConstants.RED_SHELF_X,
      VisionConstants.TOP_SHELF_Y,
      new Rotation2d(Math.PI/2)
    ), getDrivetrain()));
    kDriver.get(Button.B).whileTrue(new GoToPose(() -> new Pose2d(
      DriverStation.getAlliance()==Alliance.Blue?VisionConstants.BLUE_SHELF_X:VisionConstants.RED_SHELF_X,
      VisionConstants.BOTTOM_SHELF_Y,
      new Rotation2d(Math.PI/2)
    ), getDrivetrain()));
  }

    public GameControllerDriverConfig(SwerveDriveImpl drive, ShuffleboardTab controllerTab, boolean shuffleboardUpdates) {
        super(drive, controllerTab, shuffleboardUpdates);
    }

  private void select(int value){
    double timestamp = Timer.getFPGATimestamp();
    if(value == 0 || timestamp-selectTimestamp > 5){
      selectionValues = new int[3];
    }
    selectTimestamp = timestamp;
    for(int i = 0; i < selectionValues.length; i++){
      if(selectionValues[i] == 0){
        selectionValues[i] = value;
        if(i == 2){
          selectTimestamp = 0;
          selectedNode = new Node(DriverStation.getAlliance(), selectionValues[2], 3*selectionValues[0]+selectionValues[1]-3);
        }
        break;
      }
    }
  }

  public Pose2d getSelectedPose(){
    return selectedNode.scorePose;
  }
  
  @Override
  public double getRawForwardTranslation() {
      return kDriver.get(Axis.LEFT_Y);
  }

  @Override
  public double getRawSideTranslation() {
      return kDriver.get(Axis.LEFT_X);
  }

    @Override
    public double getRawRotation() {
        return kDriver.get(Axis.RIGHT_X);
    }

    @Override
    public double getRawHeadingAngle() {
        return Math.atan2(kDriver.get(Axis.RIGHT_X), -kDriver.get(Axis.RIGHT_Y)) - Math.PI / 2;
    }

    @Override
    public double getRawHeadingMagnitude() {
        return MathUtils.calculateHypotenuse(kDriver.get(Axis.RIGHT_X), kDriver.get(Axis.RIGHT_Y));
    }

    @Override
    public boolean getIsSlowMode() {
        return kDriver.RIGHT_TRIGGER_BUTTON.getAsBoolean();
    }

    @Override
    public boolean getIsAlign() {
        return kDriver.LEFT_TRIGGER_BUTTON.getAsBoolean();
    }
}
