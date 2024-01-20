// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.ShuffleBoard.Tabs;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.SupplierCommand;
import frc.robot.commands.auto_comm.PathPlannerCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.PathGroupLoader;
import frc.robot.util.ShuffleBoard.ShuffleBoardTabs;

/** Add your docs here. */
public class AutoTab extends ShuffleBoardTabs {

    private final SendableChooser<Command> autoCommand = new SendableChooser<>();

    Drivetrain drive;

    public AutoTab(Drivetrain drive){
        this.drive = drive;
    }
    public void createEntries(){  
        tab = Shuffleboard.getTab("Auto");
        autoCommand.setDefaultOption("Do Nothing", new PrintCommand("This will do nothing!"));
        autoCommand.addOption("Example Path", new SupplierCommand(
            ()->followPath("Example Path", true),
            drive));
        tab.add(autoCommand);
    }

    public void update(){
    }

    public SendableChooser<Command> getChooser(){
        return autoCommand;
    }

    public Command followPath(String pathName, boolean resetOdemetry){
     PathPlannerPath path = PathGroupLoader.getPathGroup(pathName);
     if (resetOdemetry){
            drive.resetOdometry(path.getPreviewStartingHolonomicPose());
          }
    return AutoBuilder.followPath(PathGroupLoader.getPathGroup(pathName));
  }

}
