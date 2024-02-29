package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.gpm.Arm;
import frc.robot.subsystems.gpm.Intake;
import frc.robot.subsystems.gpm.StorageIndex;
import frc.robot.subsystems.gpm.Intake.Mode;

public class IntakeNote extends Command{

    private final Intake intake;
    private final StorageIndex storageIndex;
    private final Arm arm;
    Timer timer = new Timer();
    Boolean detectedNote = false;

    public IntakeNote(Intake intake, StorageIndex storageIndex, Arm arm) {
        this.intake = intake;
        this.storageIndex = storageIndex;
        this.arm = arm;
        addRequirements(intake, storageIndex, arm);

       
    }

    @Override
    public void initialize() {
        detectedNote = false;
        timer.reset();
        timer.stop();
        intake.setMode(Mode.INTAKE);
        storageIndex.runIndex();
        arm.setAngle(ArmConstants.intakeSetpoint);
    }

    @Override
    public void execute(){
        storageIndex.runIndex();
        if (intake.hasNote()){
            detectedNote = true;
        }
        if(!intake.hasNote() && detectedNote){
            timer.start();
        }
    }

    @Override
    public boolean isFinished(){
        return timer.hasElapsed(0.1); 
    }

    @Override
    public void end(boolean interupted){
        intake.setMode(Mode.DISABLED);
        storageIndex.stopIndex();
        arm.setAngle(ArmConstants.stowedSetpoint);
        timer.stop();
        timer.reset();
        detectedNote = false;
    }
    
}
