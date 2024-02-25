// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.ShuffleBoard.Tabs;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.auto_comm.FollowPathCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.ShuffleBoard.ShuffleBoardTabs;

/** Add your docs here. */
public class AutoTab extends ShuffleBoardTabs {

    private final SendableChooser<Command> autoCommand = new SendableChooser<>();

    private Drivetrain drive;

    public AutoTab(Drivetrain drive){
        this.drive = drive;
    }
    
    public void createEntries(){  
        tab = Shuffleboard.getTab("Auto");
        
        autoCommand.setDefaultOption("Do Nothing", new PrintCommand("This will do nothing!"));
        autoCommand.addOption("Example Path", new FollowPathCommand("Example Path",true, drive));
        autoCommand.addOption("7 Piece Auto Realistic", new FollowPathCommand("7 Piece Auto Realistic",true, drive));
        autoCommand.addOption("7 Piece Auto", new FollowPathCommand("7 Piece Auto",true, drive));       
        autoCommand.addOption("Three Piece", new FollowPathCommand("Three Piece",true, drive));
        autoCommand.addOption("Test", new FollowPathCommand("Test",true, drive));
        autoCommand.addOption("Two Piece", new FollowPathCommand("Two Piece",true, drive));
        autoCommand.addOption("Sabotage Auto", new FollowPathCommand("Sabotage Auto",true, drive));
        autoCommand.addOption("Three Piece (L)", new FollowPathCommand("Three Piece (L)",true, drive));
        autoCommand.addOption("Two Piece (pos 2) [B]", new FollowPathCommand("Two Piece (pos 2) [B]",true, drive));
        autoCommand.addOption("Two Piece (pos 4) [B]", new FollowPathCommand("Two Piece (pos 4) [B]",true, drive));
        // Important Notes and To-Do's:
        // Fix the starting angle for shooting for 2 piece pos 4, we have to do this for all of the autos, should be a quick fix
        // Also have to add constraint zones for all autos near shooting point
        // Check robot container.java for imporant note

        //Replacement points, do not delete
        // autoCommand.addOption("lol", new FollowPathCommand("lol",true, drive));
        // autoCommand.addOption("lol", new FollowPathCommand("lol",true, drive));
        // autoCommand.addOption("lol", new FollowPathCommand("lol",true, drive));
        // autoCommand.addOption("lol", new FollowPathCommand("lol",true, drive));

        autoCommand.addOption("Accuracy", new FollowPathCommand("Accuracy",true, drive));
        autoCommand.addOption("Three Piece (L)", new FollowPathCommand("Three Piece (L)",true, drive));

        
        tab.add(autoCommand);
    }

    public void update(){
    }

    public SendableChooser<Command> getChooser(){
        return autoCommand;
    }
}
