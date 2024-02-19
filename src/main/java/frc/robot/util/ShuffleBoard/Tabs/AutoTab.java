// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.ShuffleBoard.Tabs;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
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

    public void intake(CANSparkFlex Motor1, double Speed){
        Motor1.set(Speed);
    }

    public void LineCommands() {
        new ParallelRaceGroup(
            new FollowPathCommand("Line",true, drive),
            new InstantCommand(() -> intake(new CANSparkFlex(5, MotorType.kBrushless), -0.5))
        );
    }
    
    public void createEntries(){  
        tab = Shuffleboard.getTab("Auto");
        
        autoCommand.setDefaultOption("Do Nothing", new PrintCommand("This will do nothing!"));
        autoCommand.addOption("Example Path", new FollowPathCommand("Example Path",true, drive));
        autoCommand.addOption("7 Piece Auto Realistic", new FollowPathCommand("7 Piece Auto Realistic",true, drive));
        autoCommand.addOption("7 Piece Auto", new FollowPathCommand("7 Piece Auto",true, drive));       
        autoCommand.addOption("Bottom 4 Piece 4 5 (No Shooting On The Move)", new FollowPathCommand("Bottom 4 Piece 4 5 (No Shooting On The Move)",true, drive));
        autoCommand.addOption("Dream Bottom 4 Piece 4 5", new FollowPathCommand("Dream Bottom 4 Piece 4 5",true, drive));
        autoCommand.addOption("Test", new FollowPathCommand("Test", true, drive));
        autoCommand.addOption("Test 2", new FollowPathCommand("Test 2", true, drive));
        autoCommand.addOption("Test with Rotations", new FollowPathCommand("Test", true, drive));
        autoCommand.addOption("Line", new InstantCommand(() -> LineCommands()));

        // Example of running multiple commands.
        autoCommand.addOption("Multi",
            Commands.sequence(
                new FollowPathCommand("Test", true, drive),
                new InstantCommand(() -> System.out.println("Instant command."))
            )
        );

        tab.add(autoCommand);
    }

    public void update(){
    }

    public SendableChooser<Command> getChooser(){
        return autoCommand;
    }
}