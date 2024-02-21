package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.gpm.Intake;
import frc.robot.subsystems.gpm.StorageIndex;
import frc.robot.subsystems.gpm.Intake.Mode;

public class IntakeNote extends Command{

    private final Intake m_intake;
    private final StorageIndex m_storageIndex;


    public IntakeNote(Intake intake, StorageIndex storageIndex) {
        this.m_intake = intake;
        this.m_storageIndex = storageIndex;
        addRequirements(m_intake, m_storageIndex);

       
    }

    @Override
    public void initialize() {
        // start running the motors
        m_intake.setMode(Mode.INTAKE);
        m_storageIndex.runIndex(.5);
    }

    @Override
    public void execute(){

    }

    @Override
    public boolean isFinished(){
        // TODO: this will not detect a note in simulation yet.
        return m_storageIndex.hasNote(); 
    }

    @Override
    public void end(boolean interupted){
        m_intake.setMode(Mode.DISABLED);
        m_storageIndex.stopIndex();
    }
    
}
