package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DoNothing;
import frc.robot.commands.gpm.IntakeNote;
import frc.robot.commands.gpm.PrepareShooter;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.miscConstants.VisionConstants;
import frc.robot.controls.BaseDriverConfig;
import frc.robot.controls.GameControllerDriverConfig;
import frc.robot.controls.Operator;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.gpm.Arm;
import frc.robot.subsystems.gpm.Intake;
import frc.robot.subsystems.gpm.Shooter;
import frc.robot.subsystems.gpm.StorageIndex;
import frc.robot.util.ShuffleBoard.ShuffleBoardManager;
import frc.robot.util.Vision;

import java.util.function.BooleanSupplier;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems are defined here...
  private Drivetrain drive = null;
  private Vision vision = null;
  private Arm arm = null;
  private Shooter shooter = null;
  private Intake intake = null;
  private StorageIndex index = null;

  // Controllers are defined here
  private BaseDriverConfig driver = null;
  private Operator operator =null;
  ShuffleBoardManager shuffleboardManager = null;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   * <p>
   * Different robots may have different subsystems.
   */
  public RobotContainer(RobotId robotId) {
    // dispatch on the robot
    switch (robotId) {

      case TestBed1:
        index = new StorageIndex();
        shooter = new Shooter();
        break;

      case TestBed2:
        intake = new Intake();
        index = new StorageIndex();
        SmartDashboard.putData("IntakeNote", new IntakeNote(intake, index, arm));
        break;
        
      default:
      case SwerveCompetition:
        arm = new Arm();
        intake = new Intake();
        index = new StorageIndex();
        shooter = new Shooter();

      case SwerveTest:
        vision = new Vision(VisionConstants.APRIL_TAG_CAMERAS);


        drive = new Drivetrain(vision);
        driver = new GameControllerDriverConfig(drive, vision, arm, intake, index, shooter);
        operator = new Operator(intake, arm, index, shooter, drive);

        // Detected objects need access to the drivetrain
        //DetectedObject.setDrive(drive);
        
        //SignalLogger.start();

        driver.configureControls();
        operator.configureControls();
        initializeAutoBuilder();
        drive.setDefaultCommand(new DefaultDriveCommand(drive, driver));
       // registerCommands();
       // PathGroupLoader.loadPathGroups();
 
        shuffleboardManager = new ShuffleBoardManager(drive, vision);
       // SmartDashboard.putBoolean("Index beam", index.hasNote());
        break;
      }

    // This is really annoying so it's disabled
    DriverStation.silenceJoystickConnectionWarning(true);

    // TODO: verify this claim.
    // LiveWindow is causing periodic loop overruns
    LiveWindow.disableAllTelemetry();
    LiveWindow.setEnabled(false);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command pathCommand = shuffleboardManager.getSelectedCommand();
    return pathCommand;
  }

  public void updateShuffleBoard() {
    if (shuffleboardManager != null)
      shuffleboardManager.update();
  }

  /**
   * Sets whether the drivetrain uses vision to update odometry
   */
  public void setVisionEnabled(boolean enabled) {
    if (drive != null)
      drive.setVisionEnabled(enabled);
  }

  public void initializeAutoBuilder() {
    AutoBuilder.configureHolonomic(
        () -> drive.getPose(),
        (pose) -> {
          drive.resetOdometry(pose);
        },
        () -> drive.getChassisSpeeds(),
        (chassisSpeeds) -> {
          drive.setChassisSpeeds(chassisSpeeds, false);
        },
        AutoConstants.config,
        getAllianceColorBooleanSupplier(),
        drive);
  }

  public void registerCommands() {
    NamedCommands.registerCommand("Intake_Note_1.5_Sec", new IntakeNote(intake, index, arm).withTimeout(3));
    NamedCommands.registerCommand("Outtake_Note_1.5_Sec", new SequentialCommandGroup(
      new ParallelDeadlineGroup(
      new InstantCommand(() -> drive.setChassisSpeeds(new ChassisSpeeds(), true)),
      new WaitCommand(.75)),
      new InstantCommand(()-> index.runIndex()),
      new WaitCommand(.5)));
    NamedCommands.registerCommand("Prepare Shooter", new SequentialCommandGroup(new PrepareShooter(shooter, 1750), new WaitCommand(1)));

    NamedCommands.registerCommand("Index", new DoNothing());
    NamedCommands.registerCommand("Intake", new DoNothing());
  }

  public static BooleanSupplier getAllianceColorBooleanSupplier() {
    return () -> {
      // Boolean supplier that controls when the path will be mirrored for the red
      // alliance
      // This will flip the path being followed to the red side of the field.
      // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent()) {
        return alliance.get() == DriverStation.Alliance.Red;
      }
      return false;
    };
  }
}


