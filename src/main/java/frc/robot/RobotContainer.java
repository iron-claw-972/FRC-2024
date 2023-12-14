package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.globalConst;
import frc.robot.subsystems.SubsystemFactory;
import frc.robot.subsystems.drivetrain.swerve.SwerveDrive;

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
    private final SwerveDrive drive;


    // Controllers are defined here
//    private final BaseDriverConfig driver;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer(RobotId robotId) {

        drive = (SwerveDrive) SubsystemFactory.get(SwerveDrive.class);

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
//                m_vision.setUpSmartDashboardCommandButtons(m_drive);
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
//                m_vision.setUpSmartDashboardCommandButtons(m_drive);
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

        if (globalConst.USE_TELEMETRY) loadCommandSchedulerShuffleboard();

        addTestCommands();
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoCommand.getSelected();
    }

    /**
     * Adds the test commands to shuffleboard, so they can be run that way.
     */
    public void addTestCommands() {
//        GenericEntry testEntry = testTab.add("Test Results", false).getEntry();
//        testTab.add("Blinkin Id", 0.65).getEntry();
//        testTab.add("Cancel Command", new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));
//
//        if (drive != null) {
//            drive.addTestCommands(testTab, testEntry);
//        }
//
//        if (vision != null) {
//            vision.addTestCommands(testTab, testEntry, drive);
//        }
    }


    /**
     * Loads the command scheduler shuffleboard which will add event markers whenever a command finishes, ends, or is interrupted.
     */
    public void loadCommandSchedulerShuffleboard() {
        // Set the scheduler to log Shuffleboard events for command initialize, interrupt, finish
        CommandScheduler.getInstance().onCommandInitialize(command -> Shuffleboard.addEventMarker("Command initialized", command.getName(), EventImportance.kNormal));
        CommandScheduler.getInstance().onCommandInterrupt(command -> Shuffleboard.addEventMarker("Command interrupted", command.getName(), EventImportance.kNormal));
        CommandScheduler.getInstance().onCommandFinish(command -> Shuffleboard.addEventMarker("Command finished", command.getName(), EventImportance.kNormal));
    }


    /**
     * Resets the swerve modules to their absolute positions.
     */
//    public void resetModules() {
//        drive.resetModulesToAbsolute();
//    }

    /**
     * Sets whether the drivetrain uses vision to update odometry
     */
   public void enableVision(boolean enabled) {
       drive.enableVision(enabled);
   }

}