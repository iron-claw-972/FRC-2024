package frc.robot.commands.test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.miscConstants.TestConstants;
import frc.robot.subsystems.DrivetrainImpl;
import frc.robot.subsystems.Module;
import frc.robot.util.TimeAccuracyTest;

/**
 * Attempts to set all four modules to a constant angle. Determines if the modules are able to reach the angle requested in a certain time.
 */
public class TestSteerAngle extends CommandBase {

    private final DrivetrainImpl drive;
    private final GenericEntry testEntry;
    private final TimeAccuracyTest[] timeAccuracyTests = new TimeAccuracyTest[4];

    public TestSteerAngle(DrivetrainImpl drive, GenericEntry testEntry) {
        this.drive = drive;
        this.testEntry = testEntry;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
//        drive.setAllOptimize(false);
//        drive.enableStateDeadband(false);
//        for (int i = 0; i < 4; i++) {
//            Module module = drive.getModules()[i];
//            timeAccuracyTests[i] = new TimeAccuracyTest(
//                    module::getDriveVelocityError,
//                    () -> drive.getRequestedSteerVelocity(0),
//                    TestConstants.STEER_ANGLE_ERROR,
//                    TestConstants.STEER_ANGLE_TIME_ERROR
//            );
//        }
    }

    @Override
    public void execute() {
//        drive.setModuleStates(new SwerveModuleState[]{
//                new SwerveModuleState(0,
//                        new Rotation2d(drive.getRequestedSteerAngle(drive.getModules()[0].getAngle().getRadians()))),
//                new SwerveModuleState(0,
//                        new Rotation2d(drive.getRequestedSteerAngle(drive.getModules()[1].getAngle().getRadians()))),
//                new SwerveModuleState(0,
//                        new Rotation2d(drive.getRequestedSteerAngle(drive.getModules()[2].getAngle().getRadians()))),
//                new SwerveModuleState(0,
//                        new Rotation2d(drive.getRequestedSteerAngle(drive.getModules()[3].getAngle().getRadians())))
//        }, true);
//        testEntry.setBoolean(
//                timeAccuracyTests[0].calculate() &&
//                timeAccuracyTests[1].calculate() &&
//                timeAccuracyTests[2].calculate() &&
//                timeAccuracyTests[3].calculate()
//                            );
    }

    @Override
    public void end(boolean interrupted) {
//        drive.setAllOptimize(true);
//        drive.enableStateDeadband(true);
//        drive.stop();
    }

}
