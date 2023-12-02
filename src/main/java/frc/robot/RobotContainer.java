package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.controls.BaseDriverConfig;
import frc.robot.controls.GameControllerDriverConfig;
import frc.robot.controls.PS5ControllerDriverConfig;
import frc.robot.subsystems.drive.Drivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    // Shuffleboard auto chooser
    private final SendableChooser<Command> autoCommand = new SendableChooser<>();

    //shuffleboard tabs
    // The main tab is not currently used. Delete the SuppressWarning if it is used.
    @SuppressWarnings("unused")
    private final ShuffleboardTab mainTab = Shuffleboard.getTab("Main");
    private final ShuffleboardTab drivetrainTab = Shuffleboard.getTab("Drive");
    private final ShuffleboardTab swerveModulesTab = Shuffleboard.getTab("Swerve Modules");
    private final ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");
    private final ShuffleboardTab controllerTab = Shuffleboard.getTab("Controller");
    private final ShuffleboardTab visionTab = Shuffleboard.getTab("Vision");
    private final ShuffleboardTab testTab = Shuffleboard.getTab("Test");

//    private final Vision vision;

    // The robot's subsystems are defined here...
    private final Drivetrain drive;


    // Controllers are defined here
    private final BaseDriverConfig driver;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        drive = new Drivetrain(drivetrainTab);
        driver = new GameControllerDriverConfig(drive, controllerTab, false);

        driver.configureControls();

        drive.setDefaultCommand(new DefaultDriveCommand(drive, driver));
        drivetrainTab.add("feild", drive.getFeild());
        drivetrainTab.addDouble("module1", ()->drive.getModules()[0].getAngle().getDegrees());
        drivetrainTab.addDouble("module2", ()->drive.getModules()[1].getAngle().getDegrees());
        drivetrainTab.addDouble("module3", ()->drive.getModules()[2].getAngle().getDegrees());
        drivetrainTab.addDouble("module4", ()->drive.getModules()[3].getAngle().getDegrees());



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

        autoTab.add("Auto Chooser", autoCommand);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoCommand.getSelected();
    }

    // TODO
    /**
     * Resets the swerve modules to their absolute positions.
     */
//    public void resetModules() {
//        drive.resetModulesToAbsolute();
//    }


}