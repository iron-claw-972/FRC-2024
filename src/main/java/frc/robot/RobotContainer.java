package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot.RobotId;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.auto.PathPlannerCommand;
import frc.robot.constants.Constants;
import frc.robot.constants.miscConstants.VisionConstants;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.controls.BaseDriverConfig;
import frc.robot.controls.GameControllerDriverConfig;
import frc.robot.controls.PS5ControllerDriverConfig;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.PathGroupLoader;
import frc.robot.util.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Shuffleboard auto chooser
  private final SendableChooser<Command> m_autoCommand = new SendableChooser<>();

  //shuffleboard tabs
  private final ShuffleboardTab m_mainTab = Shuffleboard.getTab("Main");
  private final ShuffleboardTab m_drivetrainTab = Shuffleboard.getTab("Drive");
  private final ShuffleboardTab m_swerveModulesTab = Shuffleboard.getTab("Swerve Modules");
  private final ShuffleboardTab m_autoTab = Shuffleboard.getTab("Auto");
  private final ShuffleboardTab m_controllerTab = Shuffleboard.getTab("Controller");
  private final ShuffleboardTab m_visionTab = Shuffleboard.getTab("Vision");
  private final ShuffleboardTab m_testTab = Shuffleboard.getTab("Test");
  private final ShuffleboardTab m_elevatorTab = Shuffleboard.getTab("Elevator");
  private final ShuffleboardTab m_intakeTab = Shuffleboard.getTab("Intake");
  private final ShuffleboardTab m_wristTab = Shuffleboard.getTab("Wrist");

  private final Vision m_vision;

  // The robot's subsystems are defined here...
  private final Drivetrain m_drive;


  // Controllers are defined here
  private final BaseDriverConfig m_driver;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(RobotId robotId) {

    // PowerDistribution m_PDModule = new PowerDistribution(1, ModuleType.kRev);
    // m_PDModule.clearStickyFaults();
    // m_PDModule.close();

    switch (robotId) {
      case SwerveCompetition:
        // Update drive constants based off of robot type
        DriveConstants.update(robotId);
        VisionConstants.update(robotId);

        m_vision = new Vision(m_visionTab, VisionConstants.kCameras);

        // Create Drivetrain
        m_drive = new Drivetrain(m_drivetrainTab, m_swerveModulesTab, m_vision);

        m_driver = new PS5ControllerDriverConfig(m_drive, m_controllerTab, false);
        // m_testController.configureControls();
        // m_manualController.configureControls();
  
        // load paths before auto starts
        PathGroupLoader.loadPathGroups();

        m_driver.configureControls();

        m_vision.setupVisionShuffleboard();
        m_driver.setupShuffleboard();

        m_drive.setDefaultCommand(new DefaultDriveCommand(m_drive, m_driver));

        break;

      case SwerveTest:
        // Update drive constants based off of robot type
        DriveConstants.update(robotId);
        VisionConstants.update(robotId);

        m_vision = new Vision(m_visionTab, VisionConstants.kCameras);

        // Create Drivetrain, because every robot will have a drivetrain
        m_drive = new Drivetrain(m_drivetrainTab, m_swerveModulesTab, m_vision);
        m_driver = new GameControllerDriverConfig(m_drive, m_controllerTab, false);

        DriverStation.reportWarning("Not registering subsystems and controls due to incorrect robot", false);

        // TODO: construct dummy subsystems so SwerveTest can run all auto routines
       
        // load paths before auto starts
        PathGroupLoader.loadPathGroups();

        m_driver.configureControls();

        m_vision.setupVisionShuffleboard();
        m_driver.setupShuffleboard();

        m_drive.setDefaultCommand(new DefaultDriveCommand(m_drive, m_driver));
        
        break;

      default:
        DriverStation.reportWarning("Not registering subsystems and controls due to incorrect robot", false);

        m_vision = null;

        m_driver = null;
        m_drive = null;

        break;
    }

    // This is really annoying so it's disabled
    DriverStation.silenceJoystickConnectionWarning(true);

    LiveWindow.disableAllTelemetry(); // LiveWindow is causing periodic loop overruns
    LiveWindow.setEnabled(false);
    
    m_autoTab.add("Auto Chooser", m_autoCommand);

    if (Constants.kUseTelemetry) loadCommandSchedulerShuffleboard();
    
    addTestCommands();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoCommand.getSelected();
  }

  /**
   * Adds the test commands to shuffleboard so they can be run that way.
   */
  public void addTestCommands() {
    GenericEntry testEntry = m_testTab.add("Test Results", false).getEntry();
    GenericEntry blinkinId = m_testTab.add("Blinkin Id",0.65).getEntry();
    m_testTab.add("Cancel Command", new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));

    if (m_drive != null) {
      m_drive.addTestCommands(m_testTab, testEntry);
      m_testTab.add("Reset Odometry to Blue Shelf", new InstantCommand(() -> m_drive.resetOdometry(VisionConstants.kBlueShelfAlignPose)));
      m_testTab.add("Reset Odometry to Red Shelf", new InstantCommand(() -> m_drive.resetOdometry(VisionConstants.kRedShelfAlignPose)));
    }

    if (m_vision != null) {
      m_vision.addTestCommands(m_testTab, testEntry, m_drive);
    }
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
  public void resetModules() {
    m_drive.resetModulesToAbsolute();
  }

  /**
   * Sets whether or not the drivetrain uses vision to update odometry
   */
  public void setVisionEnabled(boolean enabled) {
    m_drive.enableVision(enabled);
  }

}