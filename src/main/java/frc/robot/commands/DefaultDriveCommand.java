package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.controls.BaseDriverConfig;
import frc.robot.subsystems.Drivetrain;

/**
 * Default drive command. Drives robot using driver controls.
 */
public class DefaultDriveCommand extends CommandBase {
    private final Drivetrain swerve;
    private final BaseDriverConfig driver;

    public DefaultDriveCommand(
            Drivetrain swerve,
            BaseDriverConfig driver
                              ) {
        this.swerve = swerve;
        this.driver = driver;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        swerve.setStateDeadband(true);
    }

    @Override
    public void execute() {

        double forwardTranslation = driver.getForwardTranslation();
        double sideTranslation = driver.getSideTranslation();
        double rotation = driver.getRotation();

        double slowFactor = driver.getIsSlowMode() ? DriveConstants.kSlowDriveFactor : 1;

        forwardTranslation *= slowFactor;
        sideTranslation *= slowFactor;
        rotation *= driver.getIsSlowMode() ? DriveConstants.kSlowRotFactor : 1;

        int allianceReversal = DriverStation.getAlliance() == Alliance.Red ? 1 : -1;
        forwardTranslation *= allianceReversal;
        sideTranslation *= allianceReversal;

        if (driver.getIsAlign()) {
            swerve.driveHeading(
                    forwardTranslation,
                    sideTranslation,
                    (Math.abs(swerve.getYaw().getRadians()) > Math.PI / 2) ? Math.PI : 0,
                    true
                               );
        } else {
            swerve.drive(
                    forwardTranslation,
                    sideTranslation,
                    rotation,
                    true,
                    false
                        );
        }
    }
}
