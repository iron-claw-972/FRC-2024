package frc.robot.controls;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.driveComm.SetFormationX;
import frc.robot.constants.globalConst;
import frc.robot.subsystems.drivetrain.swerveDrive.swerveDriveImpl;
import frc.robot.util.MathUtils;
import lib.controllers.GameController;
import lib.controllers.GameController.Axis;
import lib.controllers.GameController.Button;

/**
 * Driver controls for the generic game controller.
 */
public class GameControllerDriverConfig extends BaseDriverConfig {

    private final GameController kDriver = new GameController(globalConst.DRIVER_JOY);

    public GameControllerDriverConfig(swerveDriveImpl drive, ShuffleboardTab controllerTab, boolean shuffleboardUpdates) {
        super(drive, controllerTab, shuffleboardUpdates);
    }

    @Override
    public void configureControls() {

        // reset the yaw forward if it hasn't been. Mainly useful for testing/driver practice
        kDriver.get(Button.START).onTrue(new InstantCommand(() -> super.getDrivetrain().setYaw(
                new Rotation2d(DriverStation.getAlliance() == Alliance.Blue ? 0 : Math.PI)
                                                                                              )));

        // set the wheels to X
        kDriver.get(Button.X).onTrue(new SetFormationX(super.getDrivetrain()));

        // Resets the modules to absolute if they are having the unresolved zeroing error
        kDriver.get(Button.A).onTrue(new InstantCommand(() -> getDrivetrain().resetModulesToAbsolute()));
    }


    @Override
    public double getRawSideTranslation() {
        return kDriver.get(Axis.LEFT_X);
    }

    @Override
    public double getRawForwardTranslation() {
        return kDriver.get(Axis.LEFT_Y);
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
