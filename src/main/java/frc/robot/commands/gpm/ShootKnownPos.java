package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmToPos;
import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.gpm.Arm;
import frc.robot.subsystems.gpm.Shooter;
import frc.robot.subsystems.gpm.StorageIndex; 

public class ShootKnownPos extends SequentialCommandGroup {
	
	/**
	* This command performs a shot from a known position
	* 
	* @param shooter      the shooter to use
	* @param arm          the arm to use
	* @param storageIndex the indexer to use
	* @param armAngle     the angle to send the arm to
	* @param shooterSpeed the speed to set the shooter to
	*/
	public ShootKnownPos(Shooter shooter, Arm arm, StorageIndex storageIndex, double armAngle, double shooterSpeed) {
		addRequirements(shooter, arm, storageIndex);

		addCommands(
			new ParallelCommandGroup(
				new ArmToPos(arm, armAngle),
				new SetShooterSpeed(shooter, shooterSpeed)),
			new IndexerFeed(storageIndex),
			new ParallelCommandGroup(
				new ArmToPos(arm, ArmConstants.stowedSetpoint),
				new PrepareShooter(shooter, 0)));

	}
	
}

