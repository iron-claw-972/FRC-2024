package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.gpm.Arm;
import frc.robot.subsystems.gpm.Intake;
import frc.robot.subsystems.gpm.StorageIndex;
import frc.robot.subsystems.gpm.Intake.Mode;
import lib.controllers.GameController;
import lib.controllers.PS5Controller;

/**
 * Command to move the arm, turn on the indexer and intake, and acquire a note.
 * <p>
 * The command may not succeed if the intake jams.
 * <p>
 * We want to add Rumble
 */
public class IntakeNote extends SequentialCommandGroup {
    /**
     * Ordinary constructor that does not rumble.
     * @param intake
     * @param storageIndex
     * @param arm
     */
    public IntakeNote(Intake intake, StorageIndex storageIndex, Arm arm) {
        addCommands(
            new MoveArm(arm, ArmConstants.intakeSetpoint),
            new IntakeNoteBasic(intake, storageIndex),
            new MoveArm(arm, ArmConstants.stowedSetpoint)
        );
    }

    /**
     * Constructor that has rumble.
     * @param intake
     * @param storageIndex
     * @param arm
     * @param gc
     */
    public IntakeNote(Intake intake, StorageIndex storageIndex, Arm arm, GameController gc) {
        addCommands(
            new MoveArm(arm, ArmConstants.intakeSetpoint),
            new IntakeNoteBasic(intake, storageIndex, gc),
            new MoveArm(arm, ArmConstants.stowedSetpoint)
        );
   }
    
}
