package frc.robot;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.miscConstants.VisionConstants;
import frc.robot.controls.BaseDriverConfig;
import frc.robot.controls.GameControllerDriverConfig;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.PathGroupLoader;
import frc.robot.util.Vision;
import frc.robot.util.ShuffleBoard.ShuffleBoardManager;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    // The robot's subsystems are defined here...
    private final Drivetrain drive;
    private final Vision vision;

    // Controllers are defined here
    private final BaseDriverConfig driver;

    ShuffleBoardManager shuffleboardManager;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        vision = new Vision(VisionConstants.CAMERAS);

        drive = new Drivetrain(vision);
        driver = new GameControllerDriverConfig(drive);

        driver.configureControls();
        initializeAutoBuilder();
        drive.setDefaultCommand(new DefaultDriveCommand(drive, driver));

        PathGroupLoader.loadPathGroups();

        shuffleboardManager = new ShuffleBoardManager(drive, vision);
         




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
        shuffleboardManager.update();
    }

    // TODO
    /**
     * Resets the swerve modules to their absolute positions.
     */
//    public void resetModules() {
//        drive.resetModulesToAbsolute();
//    }

    /**
     * Sets whether the drivetrain uses vision to update odometry
     */
   public void setVisionEnabled(boolean enabled) {
       drive.setVisionEnabled(enabled);
   }

   public void initializeAutoBuilder(){
    AutoBuilder.configureHolonomic(
      ()->drive.getPose(),
      (pose) -> {drive.resetOdometry(pose);},
      ()->drive.getChassisSpeeds(),
      (chassisSpeeds) -> {drive.setChassisSpeeds(chassisSpeeds,false);},
      AutoConstants.config,
      getAllianceColorBooleanSupplier(),
      drive
    );
   }

   public static BooleanSupplier getAllianceColorBooleanSupplier(){
    return () -> {
      // Boolean supplier that controls when the path will be mirrored for the red alliance
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