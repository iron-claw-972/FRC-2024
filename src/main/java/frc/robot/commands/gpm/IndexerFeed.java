package frc.robot.commands.gpm;

import frc.robot.subsystems.gpm.StorageIndex;

public class IndexerFeed {

    private final StorageIndex m_index;

    public IndexerFeed(StorageIndex index){
        this.m_index = index;
    }

    public void initialize (){

        m_index.ejectIntoShooter();

    }

    public void execute(){

    }

    public boolean isFinished(){

        return !m_index.hasNote();
        
    }

    public void end(boolean interupted){

        m_index.stopIndex();

    }
    
}
