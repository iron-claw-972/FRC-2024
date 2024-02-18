package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.gpm.StorageIndex;

public class OuttakeAmp extends SequentialCommandGroup {
  public OuttakeAmp(Arm arm, StorageIndex index, Shooter shooter) {
    addRequirements(arm, index, shooter);
    addCommands(
        new ArmToPos(arm, ArmConstants.ampSetpoint),
        new ParallelCommandGroup(
            new InstantCommand(() -> index.ejectAmpFront(), index),
            new InstantCommand(() -> shooter.setTargetRPM(ShooterConstants.AMP_OUTTAKE_RPM), shooter)));
  }
}
