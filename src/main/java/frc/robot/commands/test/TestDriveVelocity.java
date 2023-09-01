package frc.robot.commands.test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.miscConstants.TestConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Module;
import frc.robot.util.TimeAccuracyTest;

/**
 * Attempts to run all four modules at constant velocity. Determines if the modules are able to reach the velocity requested in a certain time.
 */
public class TestDriveVelocity extends CommandBase {

    private final Drivetrain drive;
    private final GenericEntry testEntry;
    private final TimeAccuracyTest[] timeAccuracyTests = new TimeAccuracyTest[4];

    public TestDriveVelocity(Drivetrain drive, GenericEntry testEntry) {
        this.drive = drive;
        this.testEntry = testEntry;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.setAllOptimize(false);
        for (int i = 0; i < 4; i++) {
            Module module = drive.modules[i];
            timeAccuracyTests[i] = new TimeAccuracyTest(
                    () -> module.getState().speedMetersPerSecond,
                    () -> drive.getRequestedSteerVelocity(0),
                    TestConstants.DRIVE_VELOCITY_ERROR,
                    TestConstants.DRIVE_VELOCITY_TIME_ERROR
            );
        }
    }

    @Override
    public void execute() {
        drive.setModuleStates(new SwerveModuleState[]{
                new SwerveModuleState(drive.getRequestedDriveVelocity(0), new Rotation2d(Units.degreesToRadians(135))),
                new SwerveModuleState(drive.getRequestedDriveVelocity(0), new Rotation2d(Units.degreesToRadians(45))),
                new SwerveModuleState(drive.getRequestedDriveVelocity(0), new Rotation2d(Units.degreesToRadians(225))),
                new SwerveModuleState(drive.getRequestedDriveVelocity(0), new Rotation2d(Units.degreesToRadians(315)))
        }, false);
        testEntry.setBoolean(
                timeAccuracyTests[0].calculate() &&
                timeAccuracyTests[1].calculate() &&
                timeAccuracyTests[2].calculate() &&
                timeAccuracyTests[3].calculate()
                            );
    }

    @Override
    public void end(boolean interrupted) {
        drive.setAllOptimize(true);
        drive.stop();
    }

}
