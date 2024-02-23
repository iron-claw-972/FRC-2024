package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.gpm.Arm;
import frc.robot.subsystems.gpm.Intake;
import frc.robot.subsystems.gpm.StorageIndex;
import frc.robot.subsystems.gpm.Intake.Mode;

public class IntakeNote extends Command{

    private final boolean IS_COMP_BOT = false;

    private Intake intake = null;
    private StorageIndex storageIndex = null;
    private Arm arm = null;

    public IntakeNote(Intake intake, StorageIndex storageIndex, Arm arm) {

        if (IS_COMP_BOT) {
            this.intake = intake;
            this.storageIndex = storageIndex;
            this.arm = arm;
            addRequirements(intake, storageIndex, arm);
        }
       
    }

    @Override
    public void initialize() {
        if (IS_COMP_BOT) {
            intake.setMode(Mode.INTAKE);
            storageIndex.runIndex();
            arm.setAngle(ArmConstants.intakeSetpoint);
        }
    }

    @Override
    public void execute(){

    }

    @Override
    public boolean isFinished(){
        // return storageIndex.hasNote();
        return false; 
    }

    @Override
    public void end(boolean interupted){
        if (IS_COMP_BOT) {
            intake.setMode(Mode.DISABLED);
            storageIndex.stopIndex();
            arm.setAngle(ArmConstants.stowedSetpoint);
        }
    }
    
}
