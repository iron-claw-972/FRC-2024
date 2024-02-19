package frc.robot;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.miscConstants.VisionConstants;
import frc.robot.controls.BaseDriverConfig;
import frc.robot.controls.GameControllerDriverConfig;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.gpm.Intake;
import frc.robot.subsystems.gpm.Shooter;
import frc.robot.subsystems.gpm.StorageIndex;
import frc.robot.util.PathGroupLoader;
import frc.robot.util.Vision;
import frc.robot.util.ShuffleBoard.ShuffleBoardManager;
import frc.robot.commands.gpm.IntakeNote;

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
  private Shooter shooter = null;
  private Intake intake = null;
  private StorageIndex index = null;

  // Controllers are defined here
  private BaseDriverConfig driver = null;

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
        // add some motor rpm
        SmartDashboard.setDefaultNumber("RPM top", 1500.0);
        SmartDashboard.setDefaultNumber("RPM bottom", 1500.0);
        // add shooter commands
        SmartDashboard.putData("shoot",
            new InstantCommand(() -> shooter.setTargetRPM(
                SmartDashboard.getNumber("RPM top", 1500.0),
                SmartDashboard.getNumber("RPM bottom", 1500.0))));
        SmartDashboard.putData("shoot off", new InstantCommand(() -> shooter.setTargetRPM(0)));
        break;

      case TestBed2:
        intake = new Intake();
        index = new StorageIndex();
        SmartDashboard.putData("IntakeNote", new IntakeNote(intake, index));
        break;

      default:
      case SwerveCompetition:
      case Vertigo:
      case SwerveTest:
        vision = new Vision(VisionConstants.CAMERAS);

        drive = new Drivetrain(vision);
        driver = new GameControllerDriverConfig(drive, intake, index);

        driver.configureControls();
        initializeAutoBuilder();
        drive.setDefaultCommand(new DefaultDriveCommand(drive, driver));

        PathGroupLoader.loadPathGroups();

        shuffleboardManager = new ShuffleBoardManager(drive, vision);
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
    return shuffleboardManager.getSelectedCommand();
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