package frc.robot.commands.gpm;

import frc.robot.subsystems.gpm.Shooter;

public class SetShooterSpeed {

    private final Shooter m_shooter;
    private final double targetRPM;

    public SetShooterSpeed(Shooter shooter, double targetRPM) {

        this.m_shooter = shooter;
        this.targetRPM = targetRPM;

    }

    public void initialize(){
        m_shooter.setTargetRPM(targetRPM);

    }

    public void execute(){

    }

    public boolean isFinished(){
        return m_shooter.atSetpoint();
    }

    public void end(boolean interupted){

    }
}
