package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.gpm.Shooter;

public class PrepareShooter extends Command {
	private final Shooter shooter; 
	private final double target;

	/**
	* Spins the shooter up to the target speed, and exits immediatly.
	*
	* @param shooter the shooter to spin up
	* @param target  the target speed (in m/s) to spin up to.
	* @see           frc.robot.commands.gpm.SetShooterSpeed
	*/
	public PrepareShooter(Shooter shooter, double target) {
		this.shooter = shooter;
		this.target = target;
		addRequirements(shooter);
	}

    @Override
    public void initialize() {
		this.shooter.setTargetVelocity(this.target);
    }

    @Override
    public void execute(){

    }

    @Override
    public boolean isFinished(){
		return true;
    }

    @Override
    public void end(boolean interupted){

    }
}
