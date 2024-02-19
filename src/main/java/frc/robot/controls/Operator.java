package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.gpm.Intake;
import frc.robot.subsystems.gpm.Intake.Mode;
import lib.controllers.GameController;
import lib.controllers.GameController.Button;

public class Operator {
    private GameController m_operator = new GameController(1); // TODO: Set this as a constant

    public void configureControls(Intake intake) {
        m_operator.get(Button.B).onFalse(new InstantCommand(() -> intake.setMode(Mode.DISABLED)));
        m_operator.get(Button.B).onTrue(new InstantCommand(() -> intake.setMode(Mode.INTAKE)));
    }

}
