package frc.robot;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.gpm.IntakeNote;
import frc.robot.commands.gpm.PrepareShooter;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.Constants;
import frc.robot.constants.miscConstants.VisionConstants;
import frc.robot.controls.BaseDriverConfig;
import frc.robot.controls.GameControllerDriverConfig;
import frc.robot.controls.Operator;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.gpm.Arm;
import frc.robot.subsystems.gpm.Intake;
import frc.robot.subsystems.gpm.Shooter;
import frc.robot.subsystems.gpm.StorageIndex;
import frc.robot.util.PathGroupLoader;
import frc.robot.util.Vision;
import frc.robot.util.ShuffleBoard.ShuffleBoardManager;
import lib.controllers.GameController;
import lib.controllers.GameController.Button;
import lib.controllers.Controller.RumbleStatus;

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
        // GameController kDriver = new GameController(Constants.DRIVER_JOY);
        // kDriver.get(Button.X).onTrue(new InstantCommand(() -> kDriver.setRumble(RumbleStatus.RUMBLE_ON)));
        // kDriver.get(Button.X).onFalse(new InstantCommand(() -> kDriver.setRumble(RumbleStatus.RUMBLE_OFF)));
        intake = new Intake();
        index = new StorageIndex();
        operator = new Operator(intake, index);
        operator.configureControls((x)->{
                if (x){
                    operator.kDriver.setRumble(RumbleStatus.RUMBLE_ON);
                }
                else{
                    operator.kDriver.setRumble(RumbleStatus.RUMBLE_OFF);
                }
            });
 
        SmartDashboard.putBoolean("Index beam", index.hasNote());
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
        operator.configureControls((x)->{
                if (x){
                    operator.kDriver.setRumble(RumbleStatus.RUMBLE_ON);
                    driver.kDriver.setRumble(RumbleStatus.RUMBLE_ON);
                }
                else{
                    operator.kDriver.setRumble(RumbleStatus.RUMBLE_OFF);
                    driver.kDriver.setRumble(RumbleStatus.RUMBLE_OFF);
                }
            });
        initializeAutoBuilder();
        drive.setDefaultCommand(new DefaultDriveCommand(drive, driver));
        registerCommands();
        PathGroupLoader.loadPathGroups();
 
        shuffleboardManager = new ShuffleBoardManager(drive, vision);
        SmartDashboard.putBoolean("Index beam", index.hasNote());
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

  public void registerCommands() {
    NamedCommands.registerCommand("Intake_Note_1.5_Sec", new IntakeNote(intake, index, arm).withTimeout(1)); // 3 seconds used at SVR
    // NamedCommands.registerCommand("Stop", new WaitCommand(2)); // to represent stopping for shooting 
    // Mehaan -- Consulted with Jerry, just going to use a constraint zone going at .1 which should be fine instead of stopping for the area in which we are supposed to shoot
    // NamedCommands.registerCommand("PrepareShooter", new PrepareShooter(shooter, 0));
    // NamedCommands.registerCommand("SetShooterSpeed", new SetShooterSpeed(shooter, 0));
    // NamedCommands.registerCommand("ShootKnownPos", new ShootKnownPos(shooter, arm, index, null));
    NamedCommands.registerCommand("Outtake_Note_1.5_Sec", new SequentialCommandGroup(
      new ParallelDeadlineGroup(// TODO: This will end instantly
      // TODO: Don't use setChassisSpeeds(), use drive() instead and add the drivetrain as a parameter so it is a requirement
      new InstantCommand(() -> drive.setChassisSpeeds(new ChassisSpeeds(), true)),
      new WaitCommand(.75)),
      new InstantCommand(()-> index.runIndex()),
      new WaitCommand(.5)));
      //TODO: Stop index after command finishes
    NamedCommands.registerCommand("Prepare Shooter", new SequentialCommandGroup(new PrepareShooter(shooter, 1750), new WaitCommand(1)));
  }

  public static BooleanSupplier getAllianceColorBooleanSupplier() {
    return () -> {
      // Boolean supplier that controls when the path will be mirrored for the red
      // alliance
      // This will flip the path being followed to the red side of the field.
      // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

      Optional<Alliance> alliance = DriverStation.getAlliance();
      if (alliance.isPresent()) {
        return alliance.get() == Alliance.Red;
      }
      return false;
    };
  }
}


