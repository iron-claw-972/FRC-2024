package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.miscConstants.VisionConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.gpm.Arm;
import frc.robot.subsystems.gpm.Shooter;

//00 is the bottom right corner of blue wall in m
/**
 * Shoots on the move (instantaneous velocities and pose ver.).
 */
public class Shoot extends Command {
        private final Shooter shooter;
        public final Arm arm;
        private final Drivetrain drive;

        // for testing sakes
        public double horiz_angle;
        public double vert_angle;
        public double exit_vel;
        public Pose3d displacement;
        public double v_rx;
        public double v_ry;
        private final double REST_ANGLE = .55; // TODO: determine average desirable angle in radians.
        private final double REST_VEL = 4; // TODO: determine the fastest idle note-exit velocity that won't kill the battery.

        public Shoot(Shooter shooter, Arm arm, Drivetrain drivetrain) {
                this.shooter = shooter;
                this.arm = arm;
                this.drive = drivetrain;
                addRequirements(shooter);
        }

        @Override
        public void initialize() {
                // drive.setIsAlign(true); // We are already setting drivetrain angle while the command is active. -ben
                // No need to freeze driver controls. driver is mature enough to know not to mess with shoot-while-moving.
        }

        @Override
        public void execute() {
                /* temporarily, for testing purposes.
                // TODO: make this only run once?
                Pose3d speakerPose = DriverStation.getAlliance().get() == Alliance.Red ? VisionConstants.RED_SPEAKER_POSE : VisionConstants.BLUE_SPEAKER_POSE;
                //TODO: Add this and make it change with arm angle
                double shooterHeight = .5;
                double speakerHeight = 2.055;

                // Set displacement to speaker
                displacement = new Pose3d(
                        drive.getPose().getX(),
                        drive.getPose().getY(),
                        speakerHeight-shooterHeight,
                        new Rotation3d(
                                0,
                                arm.getAngle(),
                                drive.getPose().getRotation().getRadians()))
                        .relativeTo(speakerPose);

                // Set the drivetrain velocities
                v_rx = drive.getChassisSpeeds().vxMetersPerSecond;
                v_ry = drive.getChassisSpeeds().vyMetersPerSecond;
                */
                // TODO: Figure out what v_note is empirically
                double v_note = 10;

                // X distance to speaker
                double x = Math.sqrt((displacement.getX() * displacement.getX())
                        + displacement.getY() * displacement.getY());
                // Y distance to speaker (negative intended)
                double y = -displacement.getZ();

                // Basic vertical angle calculation (static robot)
                double phi_v = Math.atan(Math.pow(v_note, 2) / 9.8 / x * (1 - Math.sqrt(1
                        + 19.6 / Math.pow(v_note, 2) * (y - 4.9 * x * x / Math.pow(v_note, 2)))));
                System.err.println("pv tracks"+phi_v);
                // Angle to goal
                // double phi_h = drivetrain.getAlignAngle();
                double phi_h = Math.asin(y / x);
                System.err.println("ph " + phi_h);

                // Random variable to hold recurring code
                double a = v_note * Math.cos(phi_v) * Math.sin(phi_h);
                double theta_h = Math.atan((a + v_ry) / (v_note * Math.cos(phi_v) * Math.cos(phi_h) + v_rx));
                double theta_v = 1 / Math.atan((a + v_ry) / (v_note * Math.sin(phi_v) * Math.cos(phi_h)));
                double v_shoot = v_note * Math.sin(phi_v) / Math.sin(theta_v);
                horiz_angle = theta_h;
                vert_angle = theta_v;
                exit_vel = v_shoot;               
                System.err.println(horiz_angle);
                System.err.println(vert_angle);
                System.err.println(exit_vel);

                arm.setAngle(theta_v);
                drive.setAlignAngle(theta_h);
                // Set the outtake velocity
                shooter.setTargetVelocity(v_shoot);


        }

        @Override
        public boolean isFinished() {
                // Finishes when the outtake no longer holds the note
                // for timed version, add timer. if it has been .5 seconds but, for whatever reason,
                        // isn't fully spun up yet, abort command (with interrupted?)
                        // or do we just send-it and shoot?
                // TODO: Maybe use motor currents?
                return shooter.atSetpoint() && true;
        }

        @Override
        public void end(boolean interrupted) {
                shooter.setTargetVelocity(REST_VEL);
                // drive.setIsAlign(false); // We are already setting drivetrain angle while the command is active. -ben
                // no need to unfreeze drive control
                drive.setAlignAngle(REST_ANGLE);
        }
}
/* ver where it assumes a set amount of time to rev up the note, and locks ALL driver input.
*
 * 
 * 
public class Shoot extends Command {

        private static final Pose3d SPEAKER_POSE = new Pose3d(0, 0, 2.055, new Rotation3d());
        private static final double HEIGHT_DIFF = 0; // shooter height - speaker height

        private final Shooter shooter;
        private final Arm arm;
        private final Drivetrain drivetrain;

        private Pose3d displacement;
        private double v_rx;
        private double v_ry;
        // TODO: determine TIME_SETUP and V_NOTE empirically.
        private final double TIME_SETUP = .5; // it takes TIME_SETUP sec to spin up motors.
        private final double V_NOTE = 10; // output speed of the note.

        public Shoot(Shooter shooter, Arm arm, Drivetrain drivetrain) {
                this.shooter = shooter;
                this.arm = arm;
                this.drivetrain = drivetrain;
                addRequirements(shooter);
        }

        @Override
        public void initialize() {
                drivetrain.setIsAlign(true);
                drive.setDefaultCommand(null); // would freeze controls
        }

        @Override
        public void execute() {
                // TODO: way to abort shot?
                // TODO: formally lock robot for this duration. coasting code done.
                
                // TODO: after this has been figured out, feed.
                // Set the drivetrain velocities
                v_rx = drivetrain.getChassisSpeeds().vxMetersPerSecond;
                v_ry = drivetrain.getChassisSpeeds().vyMetersPerSecond;
                // Set displacement to speaker
                displacement = new Pose3d(
                        drivetrain.getPose().getX()+v_rx*TIME_SETUP,
                        drivetrain.getPose().getY()+v_ry*TIME_SETUP,
                        HEIGHT_DIFF,
                        new Rotation3d(
                                0,
                                arm.getAngle(),
                                drivetrain.getPose().getRotation().getRadians()))
                        .relativeTo(SPEAKER_POSE);

                // horizontal distance to speaker
                double x = Math.sqrt((displacement.getX() * displacement.getX())
                        + displacement.getY() * displacement.getY());
                // vertical distance to speaker (negative intended)
                double y = -displacement.getZ();

                // Basic vertical angle calculation (static robot)
                double phi_v = Math.atan(Math.pow(V_NOTE, 2) / 9.8 / x * (1 - Math.sqrt(1
                        + 19.6 / Math.pow(V_NOTE, 2) * (HEIGHT_DIFF - 4.9 * x * x / Math.pow(V_NOTE, 2)))));
                // Angle to goal
                // TODO: Should we use the align angle method from the drivetrain?
                // double phi_h = drivetrain.getAlignAngle();
                double phi_h = Math.asin(y / x);

                // Random variable to hold recurring code
                double a = V_NOTE * Math.cos(phi_v) * Math.sin(phi_h);
                double theta_h = Math.atan((a + v_ry) / (V_NOTE * Math.cos(phi_v) * Math.cos(phi_h) + v_rx));
                double theta_v = 1 / Math.atan((a + v_ry) / (V_NOTE * Math.sin(phi_v) * Math.cos(phi_h)));
                double v_shoot = V_NOTE * Math.sin(phi_v) / Math.sin(theta_v);

                arm.setAngle(theta_v);
                drivetrain.driveHeading(v_rx, v_ry, theta_h, true);
                shooter.setTargetVelocity(v_shoot);


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
                drivetrain.setIsAlign(false);
        }
}

 */