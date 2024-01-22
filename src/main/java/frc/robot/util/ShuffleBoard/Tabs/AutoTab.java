// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.ShuffleBoard.Tabs;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.SupplierCommand;
import frc.robot.commands.auto_comm.FollowPathCommand;
import frc.robot.commands.auto_comm.PathPlannerCommand;
import frc.robot.constants.AutoConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.PathGroupLoader;
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
        autoCommand.addOption("Bottom 4 Piece 4 5 (No Shooting On The Move)", new FollowPathCommand("Bottom 4 Piece 4 5 (No Shooting On The Move)",true, drive));       
        autoCommand.addOption("Dream Bottom 4 Piece 4 5", new FollowPathCommand("Dream Bottom 4 Piece 4 5",true, drive));

        tab.add(autoCommand);
    }

    public void update(){
    }

    public SendableChooser<Command> getChooser(){
        return autoCommand;
    }
}
