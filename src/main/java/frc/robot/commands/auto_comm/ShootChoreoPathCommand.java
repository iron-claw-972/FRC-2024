package frc.robot.commands.auto_comm;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ArmToPos;
import frc.robot.commands.gpm.IndexerFeed;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.gpm.Arm;
import frc.robot.subsystems.gpm.StorageIndex;

public class ShootChoreoPathCommand extends SequentialCommandGroup {

    public ShootChoreoPathCommand(String pathName, Drivetrain drive, Arm arm, StorageIndex indexer) {
        var pathCommand = new ChoreoPathCommand(pathName, true, drive);

        var endPose = pathCommand.getShouldFlip()
                              ? pathCommand.trajectory.getFlippedFinalPose()
                              : pathCommand.trajectory.getFinalPose();

        // Need some way to calculate the angle of the arm to shoot into the speaker using the path's end pose
        var raiseArmCommand = new ArmToPos(arm, 0);

        var driveCommands = new ParallelCommandGroup(
                pathCommand,
                raiseArmCommand
        );

        addCommands(
                driveCommands,
                new WaitCommand(0.2),
                new IndexerFeed(indexer)
                   );
    }
}
