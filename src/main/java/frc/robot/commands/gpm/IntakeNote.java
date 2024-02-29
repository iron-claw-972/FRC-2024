package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.gpm.Arm;
import frc.robot.subsystems.gpm.Intake;
import frc.robot.subsystems.gpm.StorageIndex;
import frc.robot.subsystems.gpm.Intake.Mode;
import lib.controllers.PS5Controller;

/**
 * Command to move the arm, turn on the indexer and intake, and acquire a note.
 * <p>
 * The command may not succeed if the intake jams.
 * <p>
 * We want to add Rumble
 */
public class IntakeNote extends Command{
    // member variables that hold the subsystems
    private final Intake intake;
    private final StorageIndex storageIndex;
    private final Arm arm;
    private PS5Controller gc;

    /**
     * Ordinary constructor that does not rumble.
     * @param intake
     * @param storageIndex
     * @param arm
     */
    public IntakeNote(Intake intake, StorageIndex storageIndex, Arm arm) {
        this.intake = intake;
        this.storageIndex = storageIndex;
        this.arm = arm;
        addRequirements(intake, storageIndex, arm);
    }

    public IntakeNote(Intake intake, StorageIndex storageIndex, Arm arm, PS5Controller gc) {
        this(intake, storageIndex, arm);
        this.gc = gc;
    }

    @Override
    public void initialize() {
        intake.setMode(Mode.INTAKE);
        storageIndex.runIndex();
        arm.setAngle(ArmConstants.intakeSetpoint);
    }

    @Override
    public void execute(){

    }

    @Override
    public boolean isFinished(){
        return storageIndex.hasNote(); 
    }

    @Override
    public void end(boolean interupted){
        intake.setMode(Mode.DISABLED);
        storageIndex.stopIndex();
        arm.setAngle(ArmConstants.stowedSetpoint);
    }
    
}
