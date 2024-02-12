package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.gpm.Shooter;
import frc.robot.subsystems.gpm.Arm;

/**
 * Shoots on the move.
 */
public class Shoot extends Command {

        private static final Pose3d SPEAKER_POSE = new Pose3d(0, 0, 2.055, new Rotation3d());
        private static final double SHOOTER_HEIGHT = 0;

        private final Shooter shooter;
        private final Arm arm;
        private final Drivetrain drive;

        private Pose3d displacement;
        private double v_rx;
        private double v_ry;

        public Shoot(Shooter shooter, Arm arm, Drivetrain drivetrain) {
                this.shooter = shooter;
                this.arm = arm;
                this.drive = drivetrain;
                addRequirements(shooter);
        }

        @Override
        public void initialize() {
                drive.setIsAlign(true);
        }

        @Override
        public void execute() {
                // Set displacement to speaker
                displacement = new Pose3d(
                        drive.getPose().getX(),
                        drive.getPose().getY(),
                        SHOOTER_HEIGHT,
                        new Rotation3d(
                                0,
                                arm.getAngle(),
                                drive.getPose().getRotation().getRadians()))
                        .relativeTo(SPEAKER_POSE);

                // Set the drivetrain velocities
                v_rx = drive.getChassisSpeeds().vxMetersPerSecond;
                v_ry = drive.getChassisSpeeds().vyMetersPerSecond;

                // TODO: Figure out what v_note is empirically
                double v_note = 10;

                // X distance to speaker
                double x = Math.sqrt((displacement.getX() * displacement.getX())
                        + displacement.getY() * displacement.getY());
                // Y distance to speaker (negative intended)
                double y = -displacement.getZ();

                // Basic vertical angle calculation (static robot)
                double phi_v = Math.atan(Math.pow(v_note, 2) / 9.8 / x * (1 - Math.sqrt(1
                        + 19.6 / Math.pow(v_note, 2) * (SHOOTER_HEIGHT - 4.9 * x * x / Math.pow(v_note, 2)))));
                // Angle to goal
                // double phi_h = drivetrain.getAlignAngle();
                double phi_h = Math.asin(y / x);

                // Random variable to hold recurring code
                double a = v_note * Math.cos(phi_v) * Math.sin(phi_h);
                double theta_h = Math.atan((a + v_ry) / (v_note * Math.cos(phi_v) * Math.cos(phi_h) + v_rx));
                double theta_v = 1 / Math.atan((a + v_ry) / (v_note * Math.sin(phi_v) * Math.cos(phi_h)));
                double v_shoot = v_note * Math.sin(phi_v) / Math.sin(theta_v);

                arm.setAngle(theta_v);
                drive.setAlignAngle(theta_h);
                shooter.setTargetVelocity(v_shoot);

                // Set the outtake velocity
                // outtake.shoot();

        }

        @Override
        public boolean isFinished() {
                // Finishes when the outtake no longer holds the note
                // TODO: Maybe use a timer?
                // TODO: Maybe use motor currents?
                return shooter.atSetpoint() && true;
        }

        @Override
        public void end(boolean interrupted) {
                shooter.setTargetVelocity(0);
                drive.setIsAlign(false);
                drive.setAlignAngle(null);
        }
}
