// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.ShuffleBoard;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Vision;
import frc.robot.util.ShuffleBoard.Tabs.AutoTab;
import frc.robot.util.ShuffleBoard.Tabs.SwerveTab;
import frc.robot.util.ShuffleBoard.Tabs.VisionTab;

/** Add your docs here. */
public class ShuffleBoadManager {

    ArrayList<ShuffleBoardTabs> tabs = new ArrayList<>();
    
    Field feild;

    SwerveTab swerveTab;
    AutoTab autoTab;
    VisionTab visionTab;

    public ShuffleBoadManager(Drivetrain drive, Vision vision){
        
        swerveTab = new SwerveTab(drive);
        autoTab = new AutoTab(drive);
        visionTab = new VisionTab(drive, vision);
        tabs.add(swerveTab);
        tabs.add(autoTab);
        tabs.add(visionTab);

        for (ShuffleBoardTabs tab : tabs){
            tab.createEntries();
        }
        
        feild = new Field(drive);
    }

    public void update(){
        for (ShuffleBoardTabs tab : tabs){
            tab.update();
        }
        feild.updateFeild();
    }

    public Command getSelectedCommand(){
        return autoTab.getChooser().getSelected();
    }
}
