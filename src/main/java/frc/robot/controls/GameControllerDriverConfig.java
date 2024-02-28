package frc.robot.controls;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.Climb;
import frc.robot.commands.GoToPose;
import frc.robot.commands.Climb.Chain;
import frc.robot.commands.drive_comm.SetFormationX;
import frc.robot.commands.vision.DriveToNote;
import frc.robot.constants.Constants;
import frc.robot.constants.miscConstants.VisionConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.gpm.Arm;
import frc.robot.util.MathUtils;
import frc.robot.util.Vision;
import lib.controllers.GameController;
import lib.controllers.GameController.Axis;
import lib.controllers.GameController.Button;
import lib.controllers.GameController.DPad;

/**
 * Driver controls for the generic game controller.
 */
public class GameControllerDriverConfig extends BaseDriverConfig {
  private final GameController kDriver = new GameController(Constants.DRIVER_JOY);
  private Vision vision;
  private Arm arm;

  public GameControllerDriverConfig(Drivetrain drive, Vision vision, Arm arm) {
    super(drive);
    this.vision = vision;
    this.arm = arm;
  }

  @Override
  public void configureControls() {
    // Reset yaw to be away from driver
    kDriver.get(Button.START).onTrue(new InstantCommand(() -> super.getDrivetrain().setYaw(
        new Rotation2d(DriverStation.getAlliance().get() == Alliance.Blue ? 0 : Math.PI))));

    // set the wheels to X
    kDriver.get(Button.X).whileTrue(new SetFormationX(super.getDrivetrain()));
    // Enable state deadband after setting formation to X
    kDriver.get(Button.X).onFalse(new InstantCommand(()->getDrivetrain().setStateDeadband(true)));

    kDriver.get(Button.B).whileTrue(new DriveToNote(()->vision.getBestGamePiece(Math.PI/2), getDrivetrain()));

    // Resets the modules to absolute if they are having the unresolved zeroing
    // error
    kDriver.get(Button.A).onTrue(new InstantCommand(() -> getDrivetrain().resetModulesToAbsolute()));

    // Align to stage and climb
    kDriver.get(DPad.LEFT).toggleOnTrue(new Climb(Chain.LEFT, getDrivetrain(), arm));
    kDriver.get(DPad.UP).toggleOnTrue(new Climb(Chain.CENTER, getDrivetrain(), arm));
    kDriver.get(DPad.RIGHT).toggleOnTrue(new Climb(Chain.RIGHT, getDrivetrain(), arm));

    // Amp alignment
    kDriver.get(Button.LB)
        .whileTrue(new GoToPose(
            () -> DriverStation.getAlliance().get() == Alliance.Blue ? VisionConstants.BLUE_AMP_POSE
                : VisionConstants.RED_AMP_POSE,
            getDrivetrain()));
    // Podium alignment
    kDriver.get(Button.RB)
        .whileTrue(new GoToPose(
            () -> DriverStation.getAlliance().get() == Alliance.Blue ? VisionConstants.BLUE_PODIUM_POSE
                : VisionConstants.RED_PODIUM_POSE,
            getDrivetrain()));
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
