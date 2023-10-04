package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Vision;

/**
 * Command to test alignment using vision
 * It only works if it can see an april tag at the setpoint
 */
public class TestVisionAlignment extends CommandBase {
    private final Drivetrain drive;
    private final Vision vision;
    private final double setpoint;
    private double mostRecentAngle;

    /**
     * Constructs the TestVisionAlignment command
     *
     * @param targetAngle The setpoint, in radians, that it should align to
     * @param drive       The drivetrain
     * @param vision      The vision
     */
    public TestVisionAlignment(double targetAngle, Drivetrain drive, Vision vision) {
        addRequirements(drive);
        setpoint = targetAngle;
        this.drive = drive;
        this.vision = vision;
        mostRecentAngle = setpoint + Math.PI;
    }

    /**
     * Initializes the command
     */
    @Override
    public void initialize() {
        drive.getRotationController().reset();
    }

    /**
     * Runs the drivetrain's angle PID to get the the setpoint based on vision
     */
    @Override
    public void execute() {
        mostRecentAngle = getAngle();
        double speed = drive.getRotationController().calculate(mostRecentAngle, setpoint);
        drive.drive(0, 0, speed, false, false);
    }

    /**
     * Prints the vision pose and stops the robot
     *
     * @param interrupted If the command was interrupted
     */
    @Override
    public void end(boolean interrupted) {
        System.out.printf("\nExact angle: %.4f degrees\n", Units.radiansToDegrees(getAngle()));
        drive.stop();
    }

    /**
     * Returns if the command is finished
     *
     * @return If the rotation PID is at the setpoint
     */
    @Override
    public boolean isFinished() {
        return drive.getRotationController().atSetpoint();
    }

    /**
     * Gets the angle from vision
     *
     * @return The angle from the vision, the previous angle if it doensn't get a value
     */
    private double getAngle() {
        Pose2d pose = vision.getPose2d(drive.getPose());
        if (pose != null) {
            mostRecentAngle = pose.getRotation().getRadians();
        }
        return mostRecentAngle;
    }
}
