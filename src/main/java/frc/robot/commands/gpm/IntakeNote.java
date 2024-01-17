package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.gpm_subsystem.Intake;

public class IntakeNote extends CommandBase {

    private final Intake intake;
    private final boolean forever;

    public IntakeNote(Intake intake, boolean forever) {
        addRequirements(intake);
        this.intake = intake;
        this.forever = forever;
    }

}
