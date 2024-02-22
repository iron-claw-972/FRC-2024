// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.RumbleCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.gpm.Intake;
import frc.robot.subsystems.gpm.Intake.Mode;
import lib.controllers.GameController;
import lib.controllers.GameController.Button;

/** Add your docs here. */
public class Operator{
    
    private final GameController kController = new GameController(Constants.OPERATOR_JOY);
    
    Intake intake;
    
    public Operator(Intake intake){
        this.intake = intake;
    }

    public void configureControls(){
        intake.setJamCommand(new RumbleCommand(kController, 1));

        kController.get(Button.X).onTrue(new InstantCommand(()->intake.setMode(Mode.INTAKE)));
        kController.get(Button.X).onFalse(new InstantCommand(()->intake.setMode(Mode.DISABLED)));
        kController.get(Button.B).onTrue(new InstantCommand(()->intake.setMode(Mode.REVERSE)));
        kController.get(Button.B).onFalse(new InstantCommand(()->intake.setMode(Mode.DISABLED)));
    }
}
