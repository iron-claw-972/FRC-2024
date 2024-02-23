package frc.robot.commands.gpm;

import frc.robot.subsystems.gpm.Shooter;

public class SetShooterSpeed {

    private final Shooter m_shooter;
    private final double targetRPM;

	/**
	* Spins the shooter up to the target speed, and waits before exiting.
	*
	* @param shooter    the shooter to spin up
	* @param targetRPM  the target speed (in RPM) to spin up to.
	* @see              frc.robot.commands.gpm.PrepareShooter
	*/
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
