package frc.robot.commands.gpm;

import frc.robot.subsystems.gpm.Shooter;

public class SetShooterSpeed {

    private final Shooter m_shooter;

    public SetShooterSpeed(Shooter shooter) {

        this.m_shooter = shooter;

    }
    
    public void initialize(){
        m_shooter.setTargetRPM(1);

    }

    public void execute(){

    }

    public boolean isFinished(){
        return m_shooter.atSetpoint();
    }

    public void end(boolean interupted){

    }
}
