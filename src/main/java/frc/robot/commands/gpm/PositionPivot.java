package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.gpm_subsystem.Pivot;

public class PositionPivot extends CommandBase {

    private final Pivot pivot;

    private final Pivot.Mode mode;

    public PositionPivot(Pivot pivot, Pivot.Mode mode) {
        addRequirements(pivot);
        this.pivot = pivot;
        this.mode = mode;
    }

    @Override
    public void initialize() {
        pivot.setMode(mode);
    }

    @Override
    public boolean isFinished() {
        return pivot.reachedSetpoint();
    }

}
