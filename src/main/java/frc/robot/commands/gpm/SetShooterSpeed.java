package frc.robot.commands.gpm;

import frc.robot.subsystems.gpm.Shooter;

public class SetShooterSpeed {

    private final Shooter m_shooter;
    private final double targetSpeed;

	/**
	* Spins the shooter up to the target speed, and waits before exiting.
	*
	* @param shooter      the shooter to spin up
	* @param targetSpeed  the target speed (in m/s) to spin up to.
	* @see                frc.robot.commands.gpm.PrepareShooter
	*/
    public SetShooterSpeed(Shooter shooter, double targetSpeed) {

        m_shooter = shooter;
        this.targetSpeed = targetSpeed;

    }

    public void initialize(){
        m_shooter.setTargetVelocity(targetSpeed);

    }

    public void execute(){

    }

    public boolean isFinished(){
        return m_shooter.atSetpoint();
    }

    public void end(boolean interupted){
        if (interupted){
            m_shooter.setTargetVelocity(0);
        }
    }
}
