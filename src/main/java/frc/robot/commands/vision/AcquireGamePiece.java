package frc.robot.commands.vision;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmToPos;
import frc.robot.commands.gpm.IntakeNote;

import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.DetectedObject;
import frc.robot.subsystems.gpm.Arm;
import frc.robot.subsystems.gpm.StorageIndex;

public class AcquireGamePiece extends SequentialCommandGroup {
    /**
     * Intakes a game piece
     * 
     * @param gamePiece The supplier for the game piece to intake
     * @param drive     The drivetrain
     */
    public AcquireGamePiece(Supplier<DetectedObject> gamePiece, Drivetrain drive, Arm arm, Intake intake,
            StorageIndex index) {

        addCommands(
                new ParallelCommandGroup(
                        new DriveToNote(gamePiece, drive),
                        new ArmToPos(arm, ArmConstants.intakeSetpoint)),
                new IntakeNote(intake, index));
    }
}