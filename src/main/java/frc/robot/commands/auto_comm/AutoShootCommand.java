package frc.robot.commands.auto_comm;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.gpm.PrepareShooter;
import frc.robot.subsystems.gpm.Shooter;

public class AutoShootCommand extends ParallelCommandGroup {
    public AutoShootCommand(Shooter shooter, Command... commands) {
        super();
        addCommands(new PrepareShooter(shooter, 1750));
        
        var list = new ArrayList<Command>(Arrays.asList(commands));
        list.add(0, new WaitCommand(0.6));
        addCommands(new SequentialCommandGroup(
            list.toArray(new Command[]{})
        ));
    }
}
