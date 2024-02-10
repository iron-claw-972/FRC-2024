package frc.robot.commands.gpm;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.gpm_subsystem.Intake;

public class IntakeNote extends Command {

    private final Intake intake;
    private final Debouncer stallDebouncer;


    public IntakeNote(Intake intake, boolean forever) {
        addRequirements(intake);
        this.intake = intake;
        this.stallDebouncer = new Debouncer(IntakeConstants.INTAKE_STALL_TIME, Debouncer.DebounceType.kBoth);
    }

    @Override
    public void initialize() {
        intake.setMode(Intake.Mode.INTAKE);
    }

    // TODO: This code might need some adjusting, as the cone passes through the intake.
    @Override
    public boolean isFinished() {
        return stallDebouncer.calculate(
                intake.getCurrent() >= IntakeConstants.INTAKE_CURRENT_STOP
        );
    }

    @Override
    public void end(boolean interrupted) {
        intake.setMode(Intake.Mode.DISABLED);
    }

}
