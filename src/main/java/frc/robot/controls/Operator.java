// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controls;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.OuttakeAmp;
import frc.robot.commands.OuttakeAmpManual;
import frc.robot.commands.Shoot;
import frc.robot.commands.gpm.IntakeNote;
import frc.robot.commands.gpm.PrepareShooter;
import frc.robot.commands.gpm.ShootKnownPos;
import frc.robot.commands.gpm.ShootKnownPos.ShotPosition;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.Constants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.gpm.Arm;
import frc.robot.subsystems.gpm.Intake;
import frc.robot.subsystems.gpm.Shooter;
import frc.robot.subsystems.gpm.Intake.Mode;
import frc.robot.subsystems.gpm.StorageIndex;
import lib.controllers.GameController;
import lib.controllers.GameController.Button;

public class Operator {

    private final GameController kController = new GameController(Constants.OPERATOR_JOY);

    private Intake intake;
    private Arm arm;
    private StorageIndex index;
    private Shooter shooter;
    private Drivetrain drive;

    private boolean ampOrder = false; 

    public Operator(Intake intake, Arm arm, StorageIndex index, Shooter shooter, Drivetrain drive) {
        this.intake = intake;
        this.arm = arm;
        this.index = index;
        this.shooter = shooter;
        this.drive = drive;
        this.rumbleConsumer = rumbleConsumer;
    }

    public void configureControls() {
        if (intake != null) {
            kController.get(Button.B).onTrue(new InstantCommand(() -> intake.setMode(Mode.ReverseMotors),intake));
            kController.get(Button.B).onFalse(new InstantCommand(() -> intake.setMode(Mode.DISABLED), intake));
            
            IntakeNote intakeNote = new IntakeNote(intake, index, arm, rumbleConsumer);
            kController.get(Button.X).onTrue(intakeNote);
            kController.get(Button.X).onFalse(new InstantCommand(()->intakeNote.cancel()));
        }
        if(index != null){
            kController.get(Button.B).onTrue(new InstantCommand(() -> index.ejectBack(), index));
            kController.get(Button.B).onFalse(new InstantCommand(() -> index.stopIndex(), index));
        }
        kController.get(Button.BACK).onTrue(new InstantCommand(()->{
            if(shooter != null){
                shooter.setTargetRPM(0);
                shooter.resetPID();
            }
            if(intake != null){
                intake.setMode(Mode.DISABLED);
            }
            if(arm != null){
                arm.setAngle(ArmConstants.stowedSetpoint);
            }
            if(index != null){
                index.stopIndex();
            }
            CommandScheduler.getInstance().cancelAll();
        }));
        if (index != null && arm != null && shooter != null && drive != null) {
            getRightTrigger().onTrue(new Shoot(shooter, arm, drive, index));
        }
        if(shooter != null){
            getLeftTrigger().onTrue(new PrepareShooter(shooter, Shooter.addSlip(Shooter.shooterSpeedToRPM(ShooterConstants.SHOOT_SPEED_MPS))));
        }
        if(arm != null && shooter != null && index != null){
            kController.get(Button.Y).onTrue(new ShootKnownPos(shooter, arm, index, ShotPosition.SUBWOOFER));
            // First button press sets arm and spins shooter and second button press outtakes note
            // and returns to default state
            kController.get(Button.A).onTrue(new SequentialCommandGroup(
                                            new InstantCommand(() -> {ampOrder=!ampOrder;}),
                                            new ConditionalCommand(
                                                new OuttakeAmpManual(arm, shooter), 
                                                new OuttakeAmpManual(arm, shooter, index), 
                                                () -> ampOrder)));
        }
        if(arm != null){
            kController.get(Button.RB).onTrue(new InstantCommand(()->arm.setAngle(ArmConstants.preClimbSetpoint), arm));
            kController.get(Button.LB).onTrue(new InstantCommand(()->arm.setAngle(ArmConstants.climbSetpoint), arm));
          }
    }
    public Trigger getRightTrigger(){
        return new Trigger(kController.RIGHT_TRIGGER_BUTTON);
    }
    public Trigger getLeftTrigger(){
        return new Trigger(kController.LEFT_TRIGGER_BUTTON);
    }
    public GameController getGameController(){
        return kController;
    }
}
