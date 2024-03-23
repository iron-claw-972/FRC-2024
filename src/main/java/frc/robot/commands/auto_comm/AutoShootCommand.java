package frc.robot.commands.auto_comm;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.DoNothing;
import frc.robot.commands.gpm.PrepareShooter;
import frc.robot.subsystems.gpm.Shooter;

import java.util.ArrayList;
import java.util.Arrays;

public class AutoShootCommand extends ParallelCommandGroup {

    private static final double SHOOTER_WAIT_TIME = 2.0;

    //new ParallelDeadlineGroup(
    //                        new AutoShootCommand(shooter,
    //                                new FollowPathCommand("Two Piece (pos 4) center line", true, drive)
    //                        ),
    //                        new WaitCommand(15)
    //                ),
    //                new PrepareShooter(shooter, 0)

    public AutoShootCommand(Shooter shooter, int rpm, Command... commands) {
        super();

        var commandList = new ArrayList<>(Arrays.asList(commands));
        commandList.add(0, new WaitCommand(SHOOTER_WAIT_TIME));
        commandList.add(0, new PrepareShooter(shooter, rpm));

        addCommands(new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(commandList.toArray(new Command[]{})),
                        new WaitCommand(14.6 - SHOOTER_WAIT_TIME)
                ),
                new ConditionalCommand(
                        new PrepareShooter(shooter, 0),
                        new DoNothing(),
                        () -> shooter.getLeftMotorSpeed() > 0 && shooter.getRightMotorSpeed() > 0
                )
        ));
    }
}
