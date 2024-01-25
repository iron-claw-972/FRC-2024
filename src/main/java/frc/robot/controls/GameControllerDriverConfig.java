package frc.robot.controls;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.drive_comm.SetFormationX;
import frc.robot.constants.GlobalConst;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.gpm_subsystem.Shooter;
import frc.robot.util.MathUtils;
import lib.controllers.GameController;
import lib.controllers.GameController.Axis;
import lib.controllers.GameController.Button;

/**
 * Driver controls for the generic game controller.
 */
public class GameControllerDriverConfig extends BaseDriverConfig {

    private final GameController kDriver = new GameController(GlobalConst.DRIVER_JOY);
    private final Shooter m_shooter = new Shooter();

    public GameControllerDriverConfig(Drivetrain drive, ShuffleboardTab controllerTab, boolean shuffleboardUpdates) {
        super(drive, controllerTab, shuffleboardUpdates);
    }

    @Override
    public void configureControls() {

        // reset the yaw forward if it hasn't been. Mainly useful for testing/driver practice
        kDriver.get(Button.START).onTrue(new InstantCommand(() -> super.getDrivetrain().setYaw(
                new Rotation2d(DriverStation.getAlliance().get() == Alliance.Blue ? 0 : Math.PI)
                                                                                              )));

        // set the wheels to X
        kDriver.get(Button.X).onTrue(new SetFormationX(super.getDrivetrain()));

        // Resets the modules to absolute if they are having the unresolved zeroing error
        kDriver.get(Button.A).onTrue(new InstantCommand(() ->
                getDrivetrain().resetModulesToAbsolute()
        ));
        
       // kDriver.get(Button.B).onTrue(new RunCommand(() -> getDrivetrain().driveVortex(0.6,0.6)));

        kDriver.get(Button.Y).onTrue(new RunCommand(() -> m_shooter.enable(), m_shooter));
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
