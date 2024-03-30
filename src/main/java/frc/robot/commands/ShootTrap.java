package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.gpm.ShootKnownPos;
import frc.robot.commands.gpm.ShootKnownPos.ShotPosition;
import frc.robot.constants.miscConstants.VisionConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.gpm.Arm;
import frc.robot.subsystems.gpm.Shooter;
import frc.robot.subsystems.gpm.StorageIndex;

public class ShootTrap extends SequentialCommandGroup {
    public enum Trap{
        CENTER, LEFT, RIGHT;
    }

    private Pose2d pose;
    private final double translationTolerance = 0.02;
    private final double rotationTolerance = Units.degreesToRadians(2);

    public ShootTrap(Trap trap, Drivetrain drive, Shooter shooter, Arm arm, StorageIndex index){
        addCommands(
            new InstantCommand(()->getPose(trap)),
            new GoToPosePID(()->pose, drive).until(()->
                pose.getTranslation().getDistance(drive.getPose().getTranslation()) <= translationTolerance
                && Math.abs(pose.relativeTo(drive.getPose()).getRotation().getRadians()) <= rotationTolerance
            ),
            new ShootKnownPos(shooter, arm, index, ShotPosition.TRAP)
        );
    }

    private void getPose(Trap trap){
        boolean red = Robot.getAlliance() == Alliance.Red;
        switch(trap){
            case CENTER:
                pose = red ?
                    VisionConstants.STAGE_POSES.RED_CENTER.trap :
                    VisionConstants.STAGE_POSES.BLUE_CENTER.trap;
            case LEFT:
                pose = red ?
                    VisionConstants.STAGE_POSES.RED_LEFT.trap :
                    VisionConstants.STAGE_POSES.BLUE_LEFT.trap;
            case RIGHT:
                pose = red ?
                    VisionConstants.STAGE_POSES.RED_RIGHT.trap :
                    VisionConstants.STAGE_POSES.BLUE_RIGHT.trap;
        }
    }
}
