// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.ShooterSysIDCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.gpm.Intake;
import frc.robot.subsystems.gpm.Shooter;
import frc.robot.subsystems.gpm.Intake.Mode;
import frc.robot.util.MathUtils;
import lib.controllers.GameController;
import lib.controllers.GameController.Axis;
import lib.controllers.GameController.Button;
import lib.controllers.GameController.RumbleStatus;

/** Add your docs here. */
public class Operater{
    
    private final GameController kDriver = new GameController(Constants.OPERATOR_JOY);
    
    Intake intake;
    Shooter shooter;
    public Operater(Intake intake,Shooter shooter){
        this.intake = intake;
        this.shooter = shooter;
    }

    public void configureControls(){
        kDriver.get(Button.X).onTrue(new InstantCommand(()->intake.setMode(Mode.INTAKE)));
        kDriver.get(Button.X).onFalse(new InstantCommand(()->intake.setMode(Mode.DISABLED)));
        kDriver.get(Button.B).onTrue(new InstantCommand(()->intake.setMode(Mode.REVERSE)));
        kDriver.get(Button.B).onFalse(new InstantCommand(()->intake.setMode(Mode.DISABLED)));
    }
}
