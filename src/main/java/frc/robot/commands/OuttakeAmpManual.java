package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.StorageIndexConstants;
import frc.robot.subsystems.gpm.Arm;
import frc.robot.subsystems.gpm.Shooter;
import frc.robot.subsystems.gpm.StorageIndex;

public class OuttakeAmpManual extends SequentialCommandGroup { 

    /**
     * Sets arm to amp position and starts spinning up motors 
     * @param arm
     * @param shooter
     */
    public OuttakeAmpManual(Arm arm, Shooter shooter) { 
        addCommands(
            new ParallelCommandGroup (
                // Starts spinning up shooter
                new InstantCommand(() -> shooter.setTargetRPM(ShooterConstants.AMP_OUTTAKE_RPM), shooter)),
                // Move arm to amp position 
                new ArmToPos(arm, ArmConstants.ampSetpoint));
    }

    /**
     * Scored note into amp and then returns to default state 
     * @param index
     * @param arm
     * @param shooter
     */ 
    public OuttakeAmpManual(StorageIndex index, Arm arm, Shooter shooter) { 
        addCommands(
                // Run indexer to outtake note 
                new InstantCommand(() -> index.runIndex()),
                // Waits until note has been scored 
                new WaitCommand(StorageIndexConstants.ejectAmpFrontTimeout),
                // Set everything back to default state
                new ParallelCommandGroup ( 
                    new InstantCommand(() -> shooter.setTargetRPM(0)),
                    new InstantCommand(() -> index.stopIndex()),
                    new InstantCommand(() -> arm.setAngle(ArmConstants.stowedSetpoint))));
    }
}