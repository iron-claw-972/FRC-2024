// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.ShuffleBoard.Tabs;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DoNothing;
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
        autoCommand.setDefaultOption("Do nothing", new DoNothing());

        // 1 piece
        autoCommand.addOption("1 piece pos4", new FollowPathCommand("1 piece pos4",true, drive));
        autoCommand.addOption("1.5 piece (pos 4) (B)", new FollowPathCommand("1.5 piece (pos 4) (B)",true, drive));
        autoCommand.addOption("pos 3 stage", new FollowPathCommand("pos 3 stage",true, drive));
         autoCommand.addOption("Stage 1 piece pos4", new FollowPathCommand("Stage 1 piece pos4",true, drive));
       // 2 piece
        autoCommand.addOption("2.5 Piece (pos 2) (B)", new FollowPathCommand("2.5 Piece (pos 2) (B)",true, drive));
        autoCommand.addOption("Two Piece (pos 4) center line", new FollowPathCommand("Two Piece (pos 4) center line",true, drive));
        // 3 piece
        autoCommand.addOption("Three Piece (pos 2) (B)", new FollowPathCommand("Three Piece (pos 2) (B)",true, drive));
        autoCommand.addOption("Three Piece (pos 4) understage", new FollowPathCommand("Three Piece (pos 4) understage",true, drive));
        autoCommand.addOption("[2] Three Piece (pos 2) [B]", new FollowPathCommand("[2] Three Piece (pos 2) [B]",true, drive));
       // 4 piece
       autoCommand.addOption("4 piece (pos 3)", new FollowPathCommand("4 piece (pos 3)",true, drive));
        autoCommand.addOption("4.5 piece (pos 3) (normal)", new FollowPathCommand("4.5 piece (pos 3) (normal)",true, drive));
        // 5 piece
       
        autoCommand.addOption("5 piece (pos 3) (normal)", new FollowPathCommand("5 piece (pos 3) (normal)",true, drive));
        autoCommand.addOption("5 piece (pos 3) (test)", new FollowPathCommand("5 piece (pos 3) (test)",true, drive));
        autoCommand.addOption("5 piece (pos 3) (under the stage -- test)", new FollowPathCommand("5 piece (pos 3) (under the stage -- test)",true, drive));
       
        
        // extra
         autoCommand.addOption("God Path", new FollowPathCommand("God Path",true, drive));
        autoCommand.addOption("Test", new FollowPathCommand("Test",true, drive));
         autoCommand.addOption("Test1", new FollowPathCommand("Test1",true, drive));
        
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
