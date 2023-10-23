package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.DrivetrainImpl;
import frc.robot.util.LogManager;
import frc.robot.util.Vision;

/**
 * Calculates distance traveled from drivetrain and vision and compares the results
 */
public class TestVisionDistance extends CommandBase {
    private final DrivetrainImpl drive;
    private final Vision vision;
    private Translation2d visionStartTranslation, driveStartTranslation;
    private Pose2d currentPose = null;
    private double driveDistance;
    private double visionDistance;

    private final double speed;

    private final Timer endTimer = new Timer();
    private final Timer printTimer = new Timer();

    // How many seconds of not seeing april tag before ending the command
    private static final double END_DELAY = 0.25;

    // How many seconds between each data print
    private static final double kPrintDelay = 1;

    /**
     * Constructor for TestVisionDistance
     *
     * @param speed  What speed to move at, negative if backward
     * @param drive  The drivetrain
     * @param vision The vision
     */
    public TestVisionDistance(double speed, DrivetrainImpl drive, Vision vision) {
        addRequirements(drive);
        this.drive = drive;
        this.speed = speed;
        this.vision = vision;
    }

    /**
     * Starts the timers and disables vision for odometry
     */
    @Override
    public void initialize() {

        endTimer.reset();
        printTimer.restart();

        drive.enableVision(false);

        currentPose = vision.getPose2d(drive.getPose());
        visionStartTranslation = currentPose.getTranslation();
        driveStartTranslation = drive.getPose().getTranslation();
        driveDistance = 0;
        visionDistance = 0;
    }

    /**
     * Drives the robot, finds the pose from the drivetrain and vision, and someimes prints the distances
     */
    @Override
    public void execute() {
        drive.drive(speed, 0, 0, false, false);
        Pose2d newestPose = vision.getPose2d(currentPose, drive.getPose());

        // If the camera can see the april tag
        if (newestPose != null) {
            // update current pose
            currentPose = newestPose;
            // reset the timer
            endTimer.reset();
            // calculate distances
            driveDistance = drive.getPose().getTranslation().getDistance(driveStartTranslation);
            visionDistance = currentPose.getTranslation().getDistance(visionStartTranslation);
            // log data on shuffleboard
            vision.getVisionTestDriveEntry().setDouble(driveDistance);
            vision.getVisionTestVisionEntry().setDouble(visionDistance);
            vision.getVisionTestDiffEntry().setDouble(visionDistance - driveDistance);
            vision.getVisionTestPercentDiffEntry().setDouble((visionDistance - driveDistance) / driveDistance * 100);
            // If kPrintDelay seconds have passed, print the data
            if (printTimer.advanceIfElapsed(kPrintDelay)) {
                System.out.println("\nDrive distance: " + driveDistance);
                System.out.println("Vision distance: " + visionDistance);
                System.out.println("Difference: " + (visionDistance - driveDistance));
                System.out.println("Percent difference: " + (visionDistance - driveDistance) / driveDistance * 100);
            }
            // log data
            if (Constants.DO_LOGGING) {
                LogManager.addDouble("Vision/Distance Test/Drive Distance", driveDistance);
                LogManager.addDouble("Vision/Distance Test/Vision Distance", visionDistance);
                LogManager.addDouble("Vision/Distance Test/Vision Error Value", visionDistance - driveDistance);
                LogManager.addDouble("Vision/Distance Test/Vision Error Percentage", (visionDistance - driveDistance) / driveDistance * 100);
            }
        } else {
            // start end timer
            endTimer.start();
        }
    }

    /**
     * Re-enables vision and stops the robot
     */
    @Override
    public void end(boolean interrupted) {
        drive.enableVision(true);
        drive.stop();
    }

    /**
     * Returns if the command is finished
     *
     * @return If the end delay has elapsed
     */
    @Override
    public boolean isFinished() {
        return endTimer.hasElapsed(END_DELAY);
    }
}