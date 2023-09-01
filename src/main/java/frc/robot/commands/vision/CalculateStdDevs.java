package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.constants.miscConstants.VisionConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.LogManager;
import frc.robot.util.MathUtils;
import frc.robot.util.Vision;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;

/**
 * Calculates standard deviations for vision
 */
public class CalculateStdDevs extends CommandBase {
    private final Drivetrain drive;
    private final Vision vision;
    private ArrayList<Pose2d> poses;
    private final int arrayLength;
    private final Timer endTimer;

    /**
     * Constructor for CalculateStdDevs
     *
     * @param posesToUse the number of poses to take the standard deviation of. More poses will take more time.
     * @param drive      The drivetrain
     * @param vision     The vision
     */
    public CalculateStdDevs(int posesToUse, Drivetrain drive, Vision vision) {
        this.drive = drive;
        this.vision = vision;
        arrayLength = posesToUse;
        endTimer = new Timer();
    }

    /**
     * Resets the pose array
     */
    @Override
    public void initialize() {
        // create the ArrayList of poses to store
        // an ArrayList prevents issues if the command ends early, and makes checking if the command has finished easy
        poses = new ArrayList<Pose2d>();
        // disable the drivetrain's vision usage. The drivetrain uses vision for the odometry which is used for the reference pose
        // that can affect the vision pose slightly if we are using the REFERENCE_POSE strategy
        drive.enableVision(false);
    }

    /**
     * Adds a pose to the array
     */
    @Override
    public void execute() {
        Pose2d pose = vision.getPose2d(drive.getPose());
        // If the pose exists, add it to the first open spot in the array
        if (pose != null) {
            // if we see a pose, reset the timer (it will be started the next time it doesn't get a pose)
            endTimer.stop();
            endTimer.reset();
            // add the pose to our data
            poses.add(pose);
            System.out.printf("%.1f%% done", ((double) poses.size()) / arrayLength * 100);
        } else {
            endTimer.start();
            // If kStdDevCommandEndTime seconds have passed since it saw an April tag, stop the command
            // Prevents it from running forever
            if (endTimer.hasElapsed(VisionConstants.kStdDevCommandEndTime)) {
                cancel();
            }
        }
    }

    /**
     * Calculates the standard deviation
     */
    @Override
    public void end(boolean interrupted) {

        // re-enable vision for drivetrain odometry
        drive.enableVision(true);

        // If the array is empty, don't try to calculate std devs
        if (poses.size() == 0) {
            System.out.println("There are no poses in the array\nTry again where the robot can see an April tag.");
            return;
        }

        // create arrays of the poses by X, Y, and Rotation for calculations
        double[] xArray = new double[poses.size()];
        double[] yArray = new double[poses.size()];
        double[] rotArray = new double[poses.size()];

        // copy the values into the arrays
        for (int i = 0; i < poses.size(); i++) {
            xArray[i] = poses.get(i).getX();
            yArray[i] = poses.get(i).getY();
            rotArray[i] = poses.get(i).getRotation().getRadians();
        }

        // Calculate the standard deviations
        double stdDevX = MathUtils.stdDev(xArray);
        double stdDevY = MathUtils.stdDev(yArray);
        double stdDevRot = MathUtils.stdDev(rotArray);

        // Calculate distance to closest April tag
        double closest = 1000000;
        ArrayList<EstimatedRobotPose> estimatedPoses = vision.getEstimatedPoses(drive.getPose());
        for (EstimatedRobotPose estimatedPose : estimatedPoses) {
            for (PhotonTrackedTarget target : estimatedPose.targetsUsed) {
                double distance = target.getBestCameraToTarget().getTranslation().toTranslation2d().getNorm();
                closest = Math.min(closest, distance);
            }
        }

        // Print and log values
        System.out.printf("Standard deviation values:\nX: %.5f\nY: %.5f\nRotation: %.5f\nDistance: %.5f\n",
                          stdDevX, stdDevY, stdDevRot, closest);
        if (Constants.DO_LOGGING) {
            LogManager.addDouble("Vision/StdDevTest/StdDevX", stdDevX);
            LogManager.addDouble("Vision/StdDevTest/StdDevY", stdDevY);
            LogManager.addDouble("Vision/StdDevTest/StdDevRotation", stdDevRot);
            LogManager.addDouble("Vision/StdDevTest/Distance", closest);
        }
    }

    /**
     * Returns if the command is finished
     *
     * @return If the array is full
     */
    @Override
    public boolean isFinished() {
        return poses.size() == arrayLength;
    }
}