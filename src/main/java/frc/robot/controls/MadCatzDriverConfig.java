package frc.robot.controls;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.miscConstants.OIConstants;
import frc.robot.subsystems.Drivetrain.DriveConstants;
import frc.robot.subsystems.Drivetrain.DrivetrainImpl;
import frc.robot.subsystems.Drivetrain.SetFormationX;
import lib.controllers.MadCatzController;
import lib.controllers.MadCatzController.MadCatzAxis;
import lib.controllers.MadCatzController.MadCatzButton;

/**
 * Driver controls for the MadCatz controller.
 */
public class MadCatzDriverConfig extends BaseDriverConfig {

    private final MadCatzController kDriver = new MadCatzController(OIConstants.DRIVER_JOY);

    public MadCatzDriverConfig(DrivetrainImpl drive, ShuffleboardTab controllerTab, boolean shuffleboardUpdates) {
        super(drive, controllerTab, shuffleboardUpdates);
    }

    @Override
    public void configureControls() {
        kDriver.get(MadCatzButton.B1).whileTrue(new SetFormationX(super.getDrivetrain()));
        kDriver.get(MadCatzButton.B2).onTrue(new InstantCommand(() -> super.getDrivetrain().setYaw(DriveConstants.kStartingHeading)));
    }

    @Override
    public double getRawSideTranslation() {
        return kDriver.get(MadCatzAxis.X);
    }

    @Override
    public double getRawForwardTranslation() {
        return -kDriver.get(MadCatzAxis.Y);
    }

    @Override
    public double getRawRotation() {
        return kDriver.get(MadCatzAxis.ZROTATE);
    }

    @Override
    public double getRawHeadingAngle() {
        return kDriver.get(MadCatzAxis.ZROTATE) * Math.PI;
    }

    @Override
    public double getRawHeadingMagnitude() {
        return kDriver.get(MadCatzAxis.SLIDER);
    }

    @Override
    public boolean getIsSlowMode() {
        return kDriver.get(MadCatzButton.B6).getAsBoolean();
    }

    @Override
    public boolean getIsAlign() {
        return kDriver.get(MadCatzButton.B7).getAsBoolean();
    }

}