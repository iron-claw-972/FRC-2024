package frc.robot.commands.test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.miscConstants.TestConstants;
import frc.robot.subsystems.Drivetrain.DriveConstants;
import frc.robot.subsystems.Drivetrain.DrivetrainImpl;
import frc.robot.util.TimeAccuracyTest;

/**
 * Attempts to change the heading of the chassis. Determines if the modules are able to reach the heading requested in a certain time.
 */
public class TestHeadingPID extends CommandBase {

    private final DrivetrainImpl drive;
    private final GenericEntry testEntry;
    private TimeAccuracyTest timeAccuracyTest;

    public TestHeadingPID(DrivetrainImpl drive, GenericEntry testEntry) {
        this.drive = drive;
        this.testEntry = testEntry;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.setAllOptimize(false);
        timeAccuracyTest = new TimeAccuracyTest(
                () -> drive.getYaw().getRadians(),
                () -> drive.getRequestedHeading(0),
                TestConstants.HEADING_ERROR,
                TestConstants.HEADING_TIME_ERROR
        );
    }

    @Override
    public void execute() {
        double headingPIDOutput = drive.getRotationController().calculate(drive.getYaw().getRadians(), drive.getRequestedHeading(drive.getYaw().getRadians()));
        // headingOutput is in rad/s. Need to convert to m/s by multiplying by radius
        headingPIDOutput *= Math.sqrt(0.5) * DriveConstants.kTrackWidth;
        drive.setModuleStates(
                new SwerveModuleState[]{
                        new SwerveModuleState(headingPIDOutput, new Rotation2d(Units.degreesToRadians(135))),
                        new SwerveModuleState(headingPIDOutput, new Rotation2d(Units.degreesToRadians(45))),
                        new SwerveModuleState(headingPIDOutput, new Rotation2d(Units.degreesToRadians(225))),
                        new SwerveModuleState(headingPIDOutput, new Rotation2d(Units.degreesToRadians(315)))
                }, false);
        testEntry.setBoolean(timeAccuracyTest.calculate());
    }

    @Override
    public void end(boolean interrupted) {
        drive.setAllOptimize(true);
        drive.stop();
    }
}
