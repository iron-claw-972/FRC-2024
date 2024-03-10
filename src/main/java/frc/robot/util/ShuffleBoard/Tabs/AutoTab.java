// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.ShuffleBoard.Tabs;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DoNothing;
import frc.robot.commands.auto_comm.AutoShootCommand;
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
        // autoCommand.setDefaultOption("Do nothing", new DoNothing());

        autoCommand.setDefaultOption("2.5 Source Speaker", new AutoShootCommand(shooter, new FollowPathCommand("Two Piece (pos 4) center line", true, drive)));
        
        autoCommand.addOption("God Path", new FollowPathCommand("God Path",true, drive));


        tab.add(autoCommand);
    }

    public void update(){
    }

    public SendableChooser<Command> getChooser(){
        return autoCommand;
    }
}
