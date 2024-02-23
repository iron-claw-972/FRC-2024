package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.gpm.StorageIndex;

public class IndexerFeed extends Command {

    private final StorageIndex m_index;

    public IndexerFeed(StorageIndex index){
        this.m_index = index;
    }

	@Override
    public void initialize (){

        m_index.ejectIntoShooter();

    }

	@Override
    public void execute(){

    }

	@Override
    public boolean isFinished(){

        return !m_index.hasNote();
        
    }

	@Override
    public void end(boolean interupted){

        m_index.stopIndex();

    }
    
}
