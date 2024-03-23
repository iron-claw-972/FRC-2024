// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.ShuffleBoard.Tabs;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DoNothing;
import frc.robot.commands.auto_comm.AutoShootCommand;
import frc.robot.commands.auto_comm.ChoreoPathCommand;
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
        autoCommand.addOption("God Path", new FollowPathCommand("God Path",true, drive));
        autoCommand.addOption("Test", new FollowPathCommand("Test",true, drive));
        autoCommand.addOption("5 piece (pos 3) (normal)", new FollowPathCommand("5 piece (pos 3) (normal)",true, drive));
        autoCommand.addOption("5 piece (pos 3) (test)", new FollowPathCommand("5 piece (pos 3) (test)",true, drive));
        autoCommand.addOption("5 piece (pos 3) (under the stage -- test)", new FollowPathCommand("5 piece (pos 3) (under the stage -- test)",true, drive));
        autoCommand.addOption("Test1", new FollowPathCommand("Test1",true, drive));
        autoCommand.addOption("Sabotage Trial (1)", new FollowPathCommand("Sabotage Trial (1)",true, drive));
        autoCommand.addOption("5 piece (pos 3) (under the stage -- test)", new FollowPathCommand("5 piece (pos 3) (under the stage -- test)",true, drive));
        autoCommand.addOption("Far", new FollowPathCommand("Far",true, drive));
        autoCommand.addOption("3 Source (pos 4) (B)", new FollowPathCommand("3 Source (pos 4) (B)",true, drive));
        autoCommand.addOption("Three Source (pos 4) center line", new FollowPathCommand("Three Piece (pos 4) center line",true, drive));

        autoCommand.addOption("Choreo Center 6", new AutoShootCommand(
                shooter,
                1750,
                new ChoreoPathCommand("Center 6", true, drive)
        ));
        autoCommand.addOption("Choreo Source 3", new AutoShootCommand(
                shooter,
                1750,
                new ChoreoPathCommand("Source 3", true, drive)
        ));

        // Previous Autos (Some will keep and still have to fix) autoCommand.setDefaultOption("Do Nothing", new PrintCommand("This will do nothing!"));
        // autoCommand.addOption("Example Path", new FollowPathCommand("Example Path",true, drive));
        // autoCommand.addOption("Two Piece (R) Close Shot", new FollowPathCommand("Two Piece (R) Close Shot",true, drive));
        // autoCommand.addOption("1 point auto (R)", new FollowPathCommand("1 point auto (R)",true, drive));
        // autoCommand.addOption("Two Piece (R) Close Shot", new FollowPathCommand("Two Piece (R) Close Shot",true, drive));
        // autoCommand.addOption("Accuracy", new FollowPathCommand("Accuracy",true, drive));
        // autoCommand.addOption("Three Piece (L)", new FollowPathCommand("Three Piece (L)",true, drive));
        

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
