package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.gpm_subsystem.Outtake;
import frc.robot.subsystems.gpm_subsystem.Wrist;

public class Shoot extends Command {

    private static final Pose3d SPEAKER_POSE = new Pose3d(0, 0, 0, new Rotation3d());
    private static final double SHOOTER_HEIGHT = 0;

    private final Outtake outtake;
    private final Wrist wrist;
    private final Drivetrain drivetrain;

    private Pose3d displacement;
    private double velocityX;
    private double velocityY;

    public Shoot(Outtake outtake, Wrist wrist, Drivetrain drivetrain) {
        this.outtake = outtake;
        this.wrist = wrist;
        this.drivetrain = drivetrain;
        addRequirements(outtake);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // Set displacement to speaker
        displacement =
                new Pose3d(
                        drivetrain.getPose().getX(),
                        drivetrain.getPose().getY(),
                        SHOOTER_HEIGHT,
                        new Rotation3d(
                                drivetrain.getPose().getRotation().getCos(),
                                drivetrain.getPose().getRotation().getSin(),
                                wrist.getMeasurement()
                        )
                ).relativeTo(SPEAKER_POSE);

        // Set the drivetrain velocities
        velocityX = drivetrain.getChassisSpeeds().vxMetersPerSecond;
        velocityY = drivetrain.getChassisSpeeds().vyMetersPerSecond;
        // Correct wrist position
        // TODO: Copy formula
        double wristTheta = 0;
        wrist.setAngle(wristTheta);

        // Set the outtake velocity
        outtake.setSetpoint();

    }

    @Override
    public boolean isFinished() {
        // Finishes when the outtake no longer holds the note
        // TODO: Maybe use a timer?
        // TODO: Maybe use motor currents?
        return true;
    }

    @Override
    public void end(boolean interrupted) {

    }

}