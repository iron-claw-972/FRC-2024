package frc.robot.util.ShuffleBoard.Tabs;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.vision.AimAtTag;
import frc.robot.commands.vision.CalculateStdDevs;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Vision;
import frc.robot.util.ShuffleBoard.ShuffleBoardTabs;

/** Add your docs here. */
public class VisionTab extends ShuffleBoardTabs {

    Drivetrain drive;
    Vision vision;

    public VisionTab(Drivetrain drive, Vision vision){
        this.drive = drive;
        this.vision = vision;
    }

    public void createEntries(){
        tab = Shuffleboard.getTab("Vision");
        addCommands(tab);         
    }

    public void update(){

    }

    public void addCommands(ShuffleboardTab tab){
        tab = Shuffleboard.getTab("Vision");
        SmartDashboard.putData("Calculate vision std devs", new CalculateStdDevs(1000, vision, drive));
        tab.add("Calculate std devs", new CalculateStdDevs(1000, vision, drive));
        SmartDashboard.putData("Vision aim at tag", new AimAtTag(drive));
        tab.add("Aim at tag", new AimAtTag(drive));
    }
}
