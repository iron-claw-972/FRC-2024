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
    
        // Final Autos
        autoCommand.addOption("One Piece (pos 7) (B)", new FollowPathCommand("One Piece (pos 7) (B)",true, drive));
        autoCommand.addOption("Two Piece (pos 4) [B]", new FollowPathCommand("Two Piece (pos 4) [B]",true, drive));
        autoCommand.addOption("One Piece (pos 2) (B)", new FollowPathCommand("One Piece (pos 2) (B)",true, drive));
        autoCommand.addOption("Three Piece (pos 2) [B]", new FollowPathCommand("Three Piece (pos 2) [B]",true, drive));
        autoCommand.addOption("Two Piece (pos 2) [B]", new FollowPathCommand("Two Piece (pos 2) [B]",true, drive));
        autoCommand.addOption("0 Piece Auto (pos 1)", new FollowPathCommand("0 Piece Auto (pos 1)",true, drive));
        autoCommand.addOption("0 Auto (pos 7)", new FollowPathCommand("0 Auto (pos 7)",true, drive));
        autoCommand.addOption("[Under the stage] One Piece (pos 4) (destroy) (B2)", new FollowPathCommand("[Under the stage] One Piece (pos 4) (destroy) (B2)",true, drive));
        autoCommand.addOption("[Under the stage] One Piece (pos 4) (destroy) (B1)", new FollowPathCommand("[Under the stage] One Piece (pos 4) (destroy) (B1)",true, drive));
        autoCommand.addOption("[Around the stage] One Piece (pos 4) (destroy) (B)", new FollowPathCommand("[Around the stage] One Piece (pos 4) (destroy) (B)",true, drive));
        autoCommand.addOption("Four Piece (M)", new FollowPathCommand("Four Piece (M)",true, drive));
        autoCommand.addOption("One Piece (pos 4) (B)", new FollowPathCommand("One Piece (pos 4) (B)",true, drive));
        // autoCommand.addOption("Pos 1 Path", new FollowPathCommand("Pos 1 Path",true, drive));
        
        autoCommand.addOption("God Path", new FollowPathCommand("God Path",true, drive));

        
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
