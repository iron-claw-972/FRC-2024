package frc.robot.commands.test_comm;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.util.TimeAccuracyTest;

/**
 * Attempts to run the motors very simply
 */
public class TestMotors extends CommandBase {

    private final Drivetrain drive;
    private final GenericEntry testEntry;
    private final TimeAccuracyTest[] timeAccuracyTests = new TimeAccuracyTest[4];

    
    public TestMotors() {
        
    }

    @Override
    public void initialize() {

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
