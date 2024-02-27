// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controls;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Shoot;
import frc.robot.commands.gpm.IntakeNote;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.gpm.Arm;
import frc.robot.subsystems.gpm.Intake;
import frc.robot.subsystems.gpm.Shooter;
import frc.robot.subsystems.gpm.Intake.Mode;
import frc.robot.subsystems.gpm.StorageIndex;
import lib.controllers.GameController;
import lib.controllers.GameController.Axis;
import lib.controllers.GameController.Button;

/** Add your docs here. */
public class Operator {

    private final GameController kDriver = new GameController(Constants.OPERATOR_JOY);

    private Intake intake;
    private Arm arm;
    private StorageIndex index;
    private Shooter shooter;
    private Drivetrain drive;
    
    public Operator(Intake intake, Arm arm, StorageIndex index, Shooter shooter, Drivetrain drive) {
        this.intake = intake;
        this.arm = arm;
        this.index = index;
        this.shooter = shooter;
        this.drive = drive;
    }

    public void configureControls() {
        if (intake != null) {
            Command intakeNote = new IntakeNote(intake, index, arm);
            kDriver.get(Button.X).onTrue(intakeNote);
            kDriver.get(Button.X).onFalse(new InstantCommand(()->intakeNote.cancel()));
            kDriver.get(Button.B).onTrue(new InstantCommand(() -> intake.setMode(Mode.ReverseMotors)));
            kDriver.get(Button.B).onFalse(new InstantCommand(() -> intake.setMode(Mode.DISABLED)));
        }
        kDriver.get(Button.BACK).onTrue(new InstantCommand(()->CommandScheduler.getInstance().cancelAll()));
        if (index != null && arm != null && shooter != null && drive != null) {
            getRightTrigger().whileTrue(new Shoot(shooter, arm, drive, index));
        }
    }
    public Trigger getRightTrigger(){
        return new Trigger(kDriver.RIGHT_TRIGGER_BUTTON);
    }
        public Trigger getLeftTrigger(){
        return new Trigger(kDriver.LEFT_TRIGGER_BUTTON);
    }
}
