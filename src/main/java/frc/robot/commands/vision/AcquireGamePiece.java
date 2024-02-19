package frc.robot.commands.vision;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.DetectedObject;

public class AcquireGamePiece extends SequentialCommandGroup{
    /**
     * Intakes a game piece
     * @param gamePiece The supplier for the game piece to intake
     * @param drive The drivetrain
     */
    public AcquireGamePiece(Supplier<DetectedObject> gamePiece, Drivetrain drive){
        //TODO: Add code to move intake after deciding on a design in 2024
        addCommands(new DriveToNote(gamePiece, drive));
    }
}