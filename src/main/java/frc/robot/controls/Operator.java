// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controls;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.commands.GoToPose;
import frc.robot.commands.IntakeWithRumble;
import frc.robot.commands.OuttakeAmp;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShootLock;
import frc.robot.commands.gpm.IntakeNote;
import frc.robot.commands.gpm.PrepareShooter;
import frc.robot.commands.gpm.ShootKnownPos;
import frc.robot.commands.gpm.ShootKnownPos.ShotPosition;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.Constants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.miscConstants.VisionConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.gpm.Arm;
import frc.robot.subsystems.gpm.Intake;
import frc.robot.subsystems.gpm.Shooter;
import frc.robot.subsystems.gpm.Intake.Mode;
import frc.robot.subsystems.gpm.StorageIndex;
import lib.controllers.GameController;
import lib.controllers.GameController.Button;
import lib.controllers.GameController.DPad;
import lib.controllers.GameController.RumbleStatus;

/** Add your docs here. */
public class Operator {

    private final GameController kDriver = new GameController(Constants.OPERATOR_JOY);

    private Intake intake;
    private Arm arm;
    private StorageIndex index;
    private Shooter shooter;
    private Drivetrain drive;
    private Consumer<Boolean> consumer;
    IntakeNote intakeNote;
    
    public Operator(Intake intake, Arm arm, StorageIndex index, Shooter shooter, Drivetrain drive, Consumer<Boolean> consumer) {
        this.intake = intake;
        this.arm = arm;
        this.index = index;
        this.shooter = shooter;
        this.drive = drive;
        this.consumer = consumer;
    }

    public void configureControls() {
        if (intake != null) {
           // Command intakeNote = new IntakeNote(intake, index, arm);
          // kDriver.setRumble(RumbleStatus.RUMBLE_ON);

            kDriver.get(Button.B).onTrue(new InstantCommand(() -> intake.setMode(Mode.ReverseMotors),intake));
            kDriver.get(Button.B).onFalse(new InstantCommand(() -> intake.setMode(Mode.DISABLED), intake));
        }
        if (intake != null) {
            intakeNote = new IntakeNote(intake, index, arm, consumer);
            kDriver.get(Button.X).onTrue(intakeNote);
            kDriver.get(Button.X).onFalse(new InstantCommand(()->intakeNote.cancel()));
        //     kDriver.get(Button.B).onTrue(new InstantCommand(() -> intake.setMode(Mode.ReverseMotors),intake));
        //     kDriver.get(Button.B).onFalse(new InstantCommand(() -> intake.setMode(Mode.DISABLED), intake));
        }
        if(index != null){
            kDriver.get(Button.B).onTrue(new InstantCommand(() -> index.ejectBack(), index));
            kDriver.get(Button.B).onFalse(new InstantCommand(() -> index.stopIndex(), index));
        }
        kDriver.get(Button.BACK).onTrue(new InstantCommand(()->{
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
            kDriver.get(Button.Y).onTrue(new ShootKnownPos(shooter, arm, index, ShotPosition.SUBWOOFER));
            kDriver.get(Button.A).onTrue(new OuttakeAmp(arm, index, shooter));
        }
        if(arm != null){
            kDriver.get(Button.RB).onTrue(new InstantCommand(()->arm.setAngle(ArmConstants.preClimbSetpoint), arm));
            kDriver.get(Button.LB).onTrue(new InstantCommand(()->arm.setAngle(ArmConstants.climbSetpoint), arm));
          }
           // Align to subwoofer
    kDriver.get(DPad.LEFT).whileTrue(new GoToPose(()->
      Robot.getAlliance() == Alliance.Red ? VisionConstants.RED_SUBWOOFER_LEFT
      : VisionConstants.BLUE_SUBWOOFER_LEFT,
      drive));
    kDriver.get(DPad.UP).whileTrue(new GoToPose(()->
      Robot.getAlliance() == Alliance.Red ? VisionConstants.RED_SUBWOOFER_CENTER
      : VisionConstants.BLUE_SUBWOOFER_CENTER,
      drive));
    kDriver.get(DPad.RIGHT).whileTrue(new GoToPose(()->
      Robot.getAlliance() == Alliance.Red ? VisionConstants.RED_SUBWOOFER_RIGHT
      : VisionConstants.BLUE_SUBWOOFER_RIGHT,
      drive));

    }
    public Trigger getRightTrigger(){
        return new Trigger(kDriver.RIGHT_TRIGGER_BUTTON);
    }
    public Trigger getLeftTrigger(){
        return new Trigger(kDriver.LEFT_TRIGGER_BUTTON);
    }
    public GameController getGameController(){
        return kDriver;
    }
}
