package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.StorageIndexConstants;
import frc.robot.constants.miscConstants.VisionConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.gpm.Arm;
import frc.robot.subsystems.gpm.Shooter;
import frc.robot.subsystems.gpm.StorageIndex;

//00 is the bottom right corner of blue wall in m
/**
 * Shoots on the move (instantaneous velocities and pose ver.).
 */

// TODO make the version with delay have correct math
public class Shoot extends Command {
        private final Shooter shooter;
        public final Arm arm;
        public final Drivetrain drive;
        private final StorageIndex index;

        private final Timer shootTimer = new Timer();

        // for testing sakes
        public double horiz_angle;
        public double vert_angle;
        public double exit_vel;
        public Pose3d displacement;
        public double v_rx;
        public double v_ry;
        // TODO: put in constants for other commands to use
        private final double REST_VEL = 0; // TODO: determine the fastest idle note-exit velocity that won't kill the
                                           // battery.
        public static final double shooterHeight = ArmConstants.ARM_LENGTH*Math.sin(ArmConstants.standbySetpoint) + ArmConstants.PIVOT_HEIGHT;
        public static final double shooterOffset = ArmConstants.PIVOT_X + ArmConstants.ARM_LENGTH * Math.cos(ArmConstants.standbySetpoint);

        public Shoot(Shooter shooter, Arm arm, Drivetrain drivetrain, StorageIndex index) {
                this.shooter = shooter;
                this.arm = arm;
                this.drive = drivetrain;
                this.index = index;
                addRequirements(shooter, arm, index);
        }

        @Override
        public void initialize() {
                // Reset the timer
                shootTimer.reset();
                shootTimer.stop();
                drive.setIsAlign(true); // Enable alignment mode on the drivetrain
        }

        @Override
        public void execute() {
                // Positive x displacement means we are to the left of the speaker
                // Positive y displacement means we are below the speaker.
                // Pose3d speakerPose = DriverStation.getAlliance().isPresent() &&
                //                 Robot.getAlliance() == Alliance.Red ?
                //                 VisionConstants.RED_SPEAKER_POSE : VisionConstants.BLUE_SPEAKER_POSE;
                Pose3d speakerPose = VisionConstants.RED_SPEAKER_POSE;
                // shooterHeight and shooterOffset have an additional offset because the shooter is offset from the arm, right?
                Rotation2d driveYaw = drive.getYaw();
                // Set displacement to speaker
                displacement = new Pose3d(
                        drive.getPose().getX() + shooterOffset * driveYaw.getCos()-speakerPose.getX(),
                        drive.getPose().getY() + shooterOffset * driveYaw.getSin()-speakerPose.getY(),
                        // shooterHeight-speakerPose.getZ(),
                        Units.inchesToMeters(22.7)-speakerPose.getZ(),
                        new Rotation3d(
                        0,
                        ShooterConstants.ANGLE_OFFSET - arm.getAngleRad(),
                        Math.PI + driveYaw.getRadians()));
                        // .relativeTo(speakerPose);
                        //.times(-1);
                
                // get the drivetrain velocities
                double driveSpeed = Math.hypot(drive.getChassisSpeeds().vxMetersPerSecond, drive.getChassisSpeeds().vyMetersPerSecond);
                double heading = driveYaw.getRadians() + Math.atan2(drive.getChassisSpeeds().vyMetersPerSecond, drive.getChassisSpeeds().vxMetersPerSecond);
                v_rx = driveSpeed * Math.cos(heading);
                v_ry = driveSpeed * Math.sin(heading);
                System.err.println(displacement.getX()+" " +
                                displacement.getY()+" " +
                                displacement.getZ()+" " +
                                        v_rx+" " +
                                        v_ry+" "+
                                        heading+" "+
                                        drive.getChassisSpeeds().vxMetersPerSecond+" "+
                                        drive.getChassisSpeeds().vyMetersPerSecond
                );
                // TODO: Figure out what v_note is empirically
                double v_note = 10;

                // X distance to speaker
                double x = Math.sqrt((displacement.getX() * displacement.getX())
                                // Y distance to speaker
                                + displacement.getY() * displacement.getY());
                // height (sorry that it's called y)
                double y = displacement.getZ();
                // Basic vertical angle calculation (static robot)
                double phi_v = Math.atan(Math.pow(v_note, 2) / 9.8 / x * (1 - Math.sqrt(1
                                + 19.6 / Math.pow(v_note, 2) * (y - 4.9 * x * x / Math.pow(v_note, 2)))));
                System.err.println("*pv " + phi_v);
                // Angle to goal
                double phi_h = Math.atan(displacement.getY()/ displacement.getX());
                // flip angle
                if (displacement.getX()>=0) phi_h += Math.PI;
                System.err.println("*ph " + phi_h);
                double theta_h = Math.atan((v_note * Math.cos(phi_v) * Math.sin(phi_h) - v_ry) / (v_note * Math.cos(phi_v) * Math.cos(phi_h) - v_rx));
                // flip angle
                if (displacement.getX()>=0) theta_h += Math.PI;
                // random quirk that using -v_rx, -v_ry works instead of +v_rx, +v_ry
                // theta_h conversion (i.e. pi-theta_h if necessary)
                // if the mirrored angle is the same-ish direction??? logic may break at high
                // v_rx and v_ry but don't worry about it
                /*
                if (Math.signum(Math.sin(theta_h)) != Math.signum(Math.sin(phi_h))
                                || Math.signum(Math.cos(theta_h)) != Math.signum(Math.cos(theta_h))) {
                        theta_h += Math.PI;
                }
                */
                double theta_v = Math.atan(
                                (v_note * Math.sin(phi_v) * Math.cos(theta_h)) /
                                (v_note * Math.cos(phi_v) * Math.cos(phi_h) - v_rx));
                // also here
                double v_shoot = v_note * Math.sin(phi_v) / Math.sin(theta_v);
                horiz_angle = theta_h;
                vert_angle = theta_v;
                exit_vel = v_shoot;
                System.err.println(horiz_angle);
                System.err.println(vert_angle);
                System.err.println(exit_vel);

                arm.setAngle(ShooterConstants.ANGLE_OFFSET - theta_v);
                // theta_h is relative to the horizontal, so
                // drive.setAlignAngle switches from theta_h to pi/2-theta_h
                // depending on if it's relative to the horizontal or the vertical.
                // Drivetrain angle is relative to positive x (toward red side)

                // Sets the angle to align to for the drivetrain, uses driveHeading in DefaultDriveCommand
                drive.setAlignAngle(Math.PI + theta_h); // would only pause rotational
                // use driveheading with x, y speed (keep same), angle;
                drive.driveHeading(v_rx, v_ry, Math.PI+theta_h, true);

                // Set the outtake velocity
                shooter.setTargetVelocity(v_shoot);

                if (arm.atSetpoint() && shooter.atSetpoint() && drive.atAlignAngle()) {
                        index.ejectIntoShooter();
                        shootTimer.start();
                }
                // TODO: Else reset timer?
        }

        @Override
        public boolean isFinished() {
                return shootTimer.hasElapsed(StorageIndexConstants.ejectShootTimeout);
        }

        @Override
        public void end(boolean interrupted) {
                shooter.setTargetVelocity(REST_VEL);
                drive.setIsAlign(false); // Use normal driver controls
                arm.setAngle(ArmConstants.standbySetpoint);
                index.stopIndex();
        }
}
