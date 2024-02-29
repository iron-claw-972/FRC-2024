package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.gpm.Intake;
import frc.robot.subsystems.gpm.Intake.Mode;
import lib.controllers.GameController;
import lib.controllers.PS5Controller;
import lib.controllers.GameController.RumbleStatus;
import frc.robot.subsystems.gpm.StorageIndex;

/**
 * Intake a note assuming the arm is in the intake position.
 * This command should not be used by ordinary commands; use IntakeNote instead.
 */
public class IntakeNoteBasic extends Command{

    private final Intake intake;
    private final StorageIndex storageIndex;
    private GameController gc;
    

    public IntakeNoteBasic(Intake intake, StorageIndex storageIndex) {
        this.intake = intake;
        this.storageIndex = storageIndex;
        addRequirements(intake, storageIndex);
    }

    public IntakeNoteBasic(Intake intake, StorageIndex storageIndex, GameController gc) {
        this(intake, storageIndex);
        this.gc = gc;
    }

    /**
     * start spinning both the indexer and the intake.
     */
    @Override
    public void initialize() {
        intake.setMode(Mode.INTAKE);
        storageIndex.runIndex();
    }

    @Override 
    public void execute() {
        // if we got a note, start Rumble
        if (gc != null && storageIndex.hasNote()) {
            gc.setRumble(GameController.RumbleStatus.RUMBLE_ON);
        };
    }

    /**
     * We are finished if the indexer has a note and we are in wait.
     * We are also finished if the intake jams.
     */
    @Override
    public boolean isFinished() {
        return intake.isJammed() || storageIndex.hasNote() && intake.waiting();
     }

    @Override 
    public void end(boolean interupted) {
        intake.setMode(Mode.DISABLED);
        storageIndex.stopIndex();
        if (gc != null) {
            gc.setRumble(GameController.RumbleStatus.RUMBLE_OFF);
        }
    }
}
