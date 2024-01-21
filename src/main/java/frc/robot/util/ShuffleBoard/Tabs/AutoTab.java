// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.util.ShuffleBoard.Tabs;

// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.PrintCommand;
// import frc.robot.commands.auto_comm.PathPlannerCommand;
// import frc.robot.subsystems.Drivetrain;
// import frc.robot.util.ShuffleBoard.ShuffleBoardTabs;

// /** Add your docs here. */
// public class AutoTab extends ShuffleBoardTabs {

//     private final SendableChooser<Command> autoCommand = new SendableChooser<>();

//     Drivetrain drive;

//     public AutoTab(Drivetrain drive){
//         this.drive = drive;
//     }
//     public void createEntries(){  
//         tab = Shuffleboard.getTab("Auto");
//         autoCommand.setDefaultOption("Do Nothing", new PrintCommand("This will do nothing!"));
//         autoCommand.addOption("One Meter", new PathPlannerCommand("One Meter", 0, drive, true));
//         autoCommand.addOption("Figure 8", new PathPlannerCommand("Figure 8", 0, drive,true));
//         tab.add(autoCommand);
//     }

//     public void update(){
//     }

//     public SendableChooser<Command> getChooser(){
//         return autoCommand;
//     }

// }
