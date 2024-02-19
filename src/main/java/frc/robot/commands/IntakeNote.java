package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.gpm.Intake;
import frc.robot.subsystems.gpm.StorageIndex;

public class IntakeNote extends Command {

  private Intake m_intake;

  private StorageIndex m_index;

  public IntakeNote(Intake intake, StorageIndex index) {
    m_intake = intake;
    m_index = index;

    addRequirements(m_intake, m_index);
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.setMode(Intake.Mode.DISABLED);
    m_index.stopIndex();
  }

  @Override
  public void execute() {
    m_intake.setMode(Intake.Mode.INTAKE);
    m_index.runIndex();
  }

  @Override
  public boolean isFinished() {
    return m_index.hasNote();
  }

}
