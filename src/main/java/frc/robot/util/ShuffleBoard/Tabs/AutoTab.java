// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.ShuffleBoard.Tabs;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DoNothing;
import frc.robot.commands.auto_comm.FollowPathCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.gpm.Shooter;
import frc.robot.util.ShuffleBoard.ShuffleBoardTabs;

/** Add your docs here. */
public class AutoTab extends ShuffleBoardTabs {

    private final SendableChooser<Command> autoCommand = new SendableChooser<>();

    private Drivetrain drive;
    private Shooter shooter;

    public AutoTab(Drivetrain drive, Shooter shooter){
        this.drive = drive;
        this.shooter = shooter;
    }
    
    public void createEntries(){  
        tab = Shuffleboard.getTab("Auto");

        // Final Autos
        autoCommand.setDefaultOption("Do nothing", new DoNothing());
        autoCommand.addOption("3 piece (pos 4) [3]", new FollowPathCommand("3 piece (pos 4) [3]",true, drive));
        autoCommand.addOption("3 piece (pos 4) [V2]", new FollowPathCommand("3 piece (pos 4) [V2]",true, drive));
        autoCommand.addOption("(New Method) 3 piece (pos 4)", new FollowPathCommand("(New Method) 3 piece (pos 4)",true, drive));
        autoCommand.addOption("(Fix with same rotaionals) 3 piece (pos 4)", new FollowPathCommand("(Fix with same rotaionals) 3 piece (pos 4)",true, drive));
        autoCommand.addOption("3 piece (pos 4) (original)", new FollowPathCommand("3 piece (pos 4) (original)",true, drive));
        autoCommand.addOption("5 piece (pos 3) (original-tested)", new FollowPathCommand("5 piece (pos 3) (original-tested)",true, drive));
        autoCommand.addOption("Changed 5 piece (pos 3)", new FollowPathCommand("Changed 5 piece (pos 3)",true, drive));

  
        // Repleacement Auto, don't delete
        // autoCommand.addOption("lol", new FollowPathCommand("lol",true, drive));


        tab.add(autoCommand);
    }

    public void update(){
    }

    public SendableChooser<Command> getChooser(){
        return autoCommand;
    }
}
