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
     * Sets arm to amp position and starts to spin up shooter 
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
     * Outtakes note and returns to default state
     * @param arm
     * @param shooter
     * @param index
     */
    public OuttakeAmpManual(Arm arm, Shooter shooter, StorageIndex index) {
            addCommands(
                new InstantCommand(() -> index.runIndex(), index),
                // Waits until note has been scored 
                new WaitCommand(StorageIndexConstants.ejectAmpFrontTimeout),
                // Set everything back to default state
                new InstantCommand(() -> {
                    shooter.setTargetRPM(0);
                    index.stopIndex();
                    arm.setAngle(ArmConstants.stowedSetpoint);},
                    shooter,
                    index,
                    arm)); 
        }
    }      
