package frc.robot.commands.gpm;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmToPos;
import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.gpm.Arm;
import frc.robot.subsystems.gpm.Shooter;
import frc.robot.subsystems.gpm.StorageIndex; 

public class ShootKnownPos extends SequentialCommandGroup {
	private final Shooter shooter;
	private final Arm arm;
	private final StorageIndex storageIndex;
	private final double armAngle;
	private final double shooterSpeed;
	
	public ShootKnownPos(Shooter shooter, Arm arm, StorageIndex storageIndex, double armAngle, double shooterSpeed) {
		this.shooter = shooter;
		this.arm = arm;
		this.storageIndex = storageIndex;

		this.armAngle = armAngle;
		this.shooterSpeed = shooterSpeed;

		addRequirements(shooter, arm, storageIndex);

		addCommands(
			new ParallelCommandGroup(
				new ArmToPos(arm, armAngle),
				new SetShooterSpeed(shooter, shooterSpeed))
			new IndexerFeed(storageIndex),
			new ParallelCommandGroup(
				new ArmToPos(arm, ArmConstants.stowedSetpoint),
				new PrepareShooter(shooter, 0)));

	}
	
}

