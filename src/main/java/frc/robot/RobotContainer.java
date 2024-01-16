package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.constants.globalConst;
import frc.robot.controls.BaseDriverConfig;
import frc.robot.controls.GameControllerDriverConfig;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.PathGroupLoader;
import frc.robot.util.ShuffleBoard.ShuffleBoadManager;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    // The robot's subsystems are defined here...
    private final Drivetrain drive;


    // Controllers are defined here
    private final BaseDriverConfig driver;

    ShuffleBoadManager shuffleboardManager;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        drive = new Drivetrain();
        driver = new GameControllerDriverConfig(drive);

        driver.configureControls();

        drive.setDefaultCommand(new DefaultDriveCommand(drive, driver));

        PathGroupLoader.loadPathGroups();

        shuffleboardManager = new ShuffleBoadManager(drive);




//        switch (robotId) {
//            case SwerveCompetition:
//                // Update drive constants based off of robot type
//                DriveConstants.update(robotId);
//                VisionConstants.update(robotId);
//
//                vision = new Vision(visionTab, VisionConstants.kCameras);
//
//                // Create Drivetrain
//                drive = new Drivetrain(drivetrainTab, swerveModulesTab, vision);
//
//                driver = new PS5ControllerDriverConfig(drive, controllerTab, false);
//                // testController.configureControls();
//                // manualController.configureControls();
//
//                // load paths before auto starts
//                PathGroupLoader.loadPathGroups();
//
//                driver.configureControls();
//
//                vision.setupVisionShuffleboard();
//                driver.setupShuffleboard();
//
//                drive.setDefaultCommand(new DefaultDriveCommand(drive, driver));
//
//                break;
//
//            case SwerveTest:
//                // Update drive constants based off of robot type
//                DriveConstants.update(robotId);
//                VisionConstants.update(robotId);
//
//                vision = new Vision(visionTab, VisionConstants.kCameras);
//
//                // Create Drivetrain, because every robot will have a drivetrain
//                drive = new Drivetrain(drivetrainTab, swerveModulesTab, vision);
//                driver = new GameControllerDriverConfig(drive, controllerTab, false);
//
//                DriverStation.reportWarning("Not registering subsystems and controls due to incorrect robot", false);
//
//                // TODO: construct dummy subsystems so SwerveTest can run all auto routines
//
//                // load paths before auto starts
//                PathGroupLoader.loadPathGroups();
//
//                driver.configureControls();
//
//                vision.setupVisionShuffleboard();
//                driver.setupShuffleboard();
//
//                drive.setDefaultCommand(new DefaultDriveCommand(drive, driver));
//
//                break;
//
//            default:
//                DriverStation.reportWarning("Not registering subsystems and controls due to incorrect robot", false);
//
//                vision = null;
//
//                driver = null;
//                drive = null;
//
//                break;
//        }

        // This is really annoying so it's disabled
        DriverStation.silenceJoystickConnectionWarning(true);

        LiveWindow.disableAllTelemetry(); // LiveWindow is causing periodic loop overruns
        LiveWindow.setEnabled(false);

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return shuffleboardManager.getSelectedCommand();
    }

    public void updateShuffleBoard(){
        if (globalConst.USE_TELEMETRY){
            shuffleboardManager.update();
        }
    }
}