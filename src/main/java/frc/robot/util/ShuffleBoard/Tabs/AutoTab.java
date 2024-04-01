// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.ShuffleBoard.Tabs;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.ArmToPos;
import frc.robot.commands.auto_comm.ChoreoPathCommand;
import frc.robot.commands.auto_comm.MoveArmForShoot;
import frc.robot.commands.gpm.IntakeNote;
import frc.robot.commands.gpm.PrepareShooter;
import frc.robot.commands.gpm.SetShooterSpeed;
import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.gpm.Arm;
import frc.robot.subsystems.gpm.Intake;
import frc.robot.subsystems.gpm.Shooter;
import frc.robot.subsystems.gpm.StorageIndex;
import frc.robot.util.ShuffleBoard.ShuffleBoardTabs;

/** Add your docs here. */
public class AutoTab extends ShuffleBoardTabs {

    private static final double SHOOTER_SPINUP_TIME = 0.4;

    private final SendableChooser<Command> autoCommand = new SendableChooser<>();

    private Drivetrain drive;
    private Shooter shooter;
    private Arm arm;
    private StorageIndex indexer;
    private Intake intake;

    public AutoTab(Drivetrain drive, Shooter shooter, Arm arm, StorageIndex indexer, Intake intake){
        this.drive = drive;
        this.shooter = shooter;
        this.arm = arm;
        this.indexer = indexer;
        this.intake = intake;
    }
    
    public void createEntries(){  
        tab = Shuffleboard.getTab("Auto");

        autoCommand.addOption("Choreo Center 6",
                new SequentialCommandGroup(
                        prepare(),
                        index(),

                        intakeAndSubwooferShot("Center 6.1"),
                        intakeAndSubwooferShot("Center 6.2"),
                        intakeAndSubwooferShot("Center 6.3"),
                        intakeAndSubwooferShot("Center 6.4"),
                        intakeAndSubwooferShot("Center 6.5")
                ));

        autoCommand.addOption("Choreo Source 4", new SequentialCommandGroup(
                prepare(),
                index(),

                intakeAndSubwooferShot("Source 4.1"),
                intakeAndSubwooferShot("Source 4.2"),
                intakeAndSubwooferShot("Source 4.3")
        ));

//        autoCommand.addOption("Choreo Distance Center 6", new SequentialCommandGroup(
//                prepare(),
//                index(),
//
//                new SetShooterSpeed(shooter, 18),
//
//                intakeAndDistanceShot("Distance Center 6.1"),
//                intakeAndDistanceShot("Distance Center 6.2"),
//                intakeAndDistanceShot("Distance Center 6.3"),
//                intakeAndDistanceShot("Distance Center 6.4"),
//                intakeAndDistanceShot("Distance Center 6.5")
//        ));

//        autoCommand.addOption("Choreo Distance Center 6 Wing First", new SequentialCommandGroup(
//                prepare(),
//                index(),
//
//                new SetShooterSpeed(shooter, 18),
//
//                intakeAndDistanceShot("Distance Center 6 Wing First.1"),
//                intakeAndDistanceShot("Distance Center 6 Wing First.2"),
//                intakeAndDistanceShot("Distance Center 6 Wing First.3"),
//                intakeAndDistanceShot("Distance Center 6 Wing First.4"),
//                intakeAndDistanceShot("Distance Center 6 Wing First.5")
//        ));

        autoCommand.addOption("Choreo Distance Center 6 Wing First Subwoofer", new SequentialCommandGroup(
                prepare(),
                index(),

                intakeAndSubwooferShot("Distance Center 6 Wing First Subwoofer.1"),
                intakeAndSubwooferShot("Distance Center 6 Wing First Subwoofer.2"),

                new SetShooterSpeed(shooter, 18),

                intakeAndDistanceShot("Distance Center 6 Wing First Subwoofer.3"),
                intakeAndDistanceShot("Distance Center 6 Wing First Subwoofer.4"),
                intakeAndDistanceShot("Distance Center 6 Wing First Subwoofer.5")
        ));

//        autoCommand.addOption("Choreo Distance Center 7", new SequentialCommandGroup(
//                prepare(),
//                index(),
//
//                new SetShooterSpeed(shooter, 18),
//
//                intakeAndDistanceShot("Distance Center 7.1"),
//                intakeAndDistanceShot("Distance Center 7.2"),
//                intakeAndDistanceShot("Distance Center 7.3"),
//                intakeAndDistanceShot("Distance Center 7.4"),
//                intakeAndDistanceShot("Distance Center 7.5"),
//                intakeAndDistanceShot("Distance Center 7.6")
//        ));

        autoCommand.addOption("Choreo Distance Source 4", new SequentialCommandGroup(
                prepare(),
                index(),

                new SetShooterSpeed(shooter, 18),

                intakeAndDistanceShot("Distance Source 4.1"),
                intakeAndDistanceShot("Distance Source 4.2"),

                new ParallelCommandGroup(
                        new ArmToPos(arm, ArmConstants.stowedSetpoint)
                                .andThen(new IntakeNote(intake, indexer, arm, (ignored) -> {})),
                        new ChoreoPathCommand("Distance Source 4.3", true, drive)
                ),
                new ParallelCommandGroup(
                        new MoveArmForShoot("Distance Source 4.4", arm),
                        new ChoreoPathCommand("Distance Source 4.4", true, drive)
                                .andThen(index())
                )
        ));

        // Final Autos
//        autoCommand.setDefaultOption("Do nothing", new DoNothing());
//        autoCommand.addOption("God Path", new FollowPathCommand("God Path",true, drive));
//        autoCommand.addOption("Test", new FollowPathCommand("Test",true, drive));
//        autoCommand.addOption("5 piece (pos 3) (normal)", new FollowPathCommand("5 piece (pos 3) (normal)",true, drive));
//        autoCommand.addOption("5 piece (pos 3) (test)", new FollowPathCommand("5 piece (pos 3) (test)",true, drive));
//        autoCommand.addOption("5 piece (pos 3) (under the stage -- test)", new FollowPathCommand("5 piece (pos 3) (under the stage -- test)",true, drive));
//        autoCommand.addOption("Test1", new FollowPathCommand("Test1",true, drive));
//        autoCommand.addOption("Sabotage Trial (1)", new FollowPathCommand("Sabotage Trial (1)",true, drive));
//        autoCommand.addOption("5 piece (pos 3) (under the stage -- test)", new FollowPathCommand("5 piece (pos 3) (under the stage -- test)",true, drive));
//        autoCommand.addOption("Far", new FollowPathCommand("Far",true, drive));
//        autoCommand.addOption("3 Source (pos 4) (B)", new FollowPathCommand("3 Source (pos 4) (B)",true, drive));
//        autoCommand.addOption("Three Source (pos 4) center line", new FollowPathCommand("Three Piece (pos 4) center line",true, drive));
//        autoCommand.addOption("2.5 Piece (pos 2) (B)", new FollowPathCommand("2.5 Piece (pos 2) (B)",true, drive));

//         Previous Autos (Some will keep and still have to fix) autoCommand.setDefaultOption("Do Nothing", new PrintCommand("This will do nothing!"));
//         autoCommand.addOption("Example Path", new FollowPathCommand("Example Path",true, drive));
//         autoCommand.addOption("Two Piece (R) Close Shot", new FollowPathCommand("Two Piece (R) Close Shot",true, drive));
//         autoCommand.addOption("1 point auto (R)", new FollowPathCommand("1 point auto (R)",true, drive));
//         autoCommand.addOption("Two Piece (R) Close Shot", new FollowPathCommand("Two Piece (R) Close Shot",true, drive));
//         autoCommand.addOption("Accuracy", new FollowPathCommand("Accuracy",true, drive));
//         autoCommand.addOption("Three Piece (L)", new FollowPathCommand("Three Piece (L)",true, drive));
        

//         Repleacement Auto, don't delete
//         autoCommand.addOption("lol", new FollowPathCommand("lol",true, drive));


        tab.add(autoCommand);
    }

    public void update(){
    }

    public SendableChooser<Command> getChooser(){
        return autoCommand;
    }

    private ParallelCommandGroup intakeAndSubwooferShot(String pathName) {
        return new ParallelCommandGroup(
                new IntakeNote(intake, indexer, arm, (ignored) -> {}),
                new ChoreoPathCommand(pathName, true, drive)
                        .andThen(index())
        );
    }

    private ParallelCommandGroup intakeAndDistanceShot(String pathName) {
        return new ParallelCommandGroup(
                new ArmToPos(arm, ArmConstants.stowedSetpoint)
                        .andThen(new IntakeNote(intake, indexer, arm, (ignored) -> {}))
                        .andThen(new MoveArmForShoot(pathName, arm)),
                new ChoreoPathCommand(pathName, true, drive)
                        .andThen(index())
        );
    }

    private Command index() {
        return new RunCommand(() -> indexer.ejectIntoShooter())
                       .withTimeout(0.1)
                       .andThen(new InstantCommand(() -> indexer.stopIndex()));
    }

    private Command prepare() {
        return new SequentialCommandGroup(
                new PrepareShooter(shooter, 1800),
                new WaitCommand(SHOOTER_SPINUP_TIME)
        );
    }
}
