package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.gpm.Arm;

public class Climb extends SequentialCommandGroup {
    public enum Chain{
        LEFT, RIGHT, CENTER;
    }

    private Pose2d pose1;
    private Pose2d pose2;
    private double tolerance1 = Units.inchesToMeters(2);
    private double tolerance2 = Units.inchesToMeters(2);

    public Climb(Chain chain, Drivetrain drive, Arm arm){
        addCommands(
            new InstantCommand(()->getPoses(chain)),
            new ParallelCommandGroup(
                new ArmToPos(arm, ArmConstants.climbSetpoint),
                new GoToPose(()->pose1, drive).until(
                    ()->drive.getPose().getTranslation().getDistance(pose1.getTranslation())<tolerance1)
            ),
            new GoToPose(()->pose2, drive).until(
                ()->drive.getPose().getTranslation().getDistance(pose2.getTranslation())<tolerance2),
            new ArmToPos(arm, ArmConstants.stowedSetpoint)
        );
    }

    private void getPoses(Chain chain){

    }
}
