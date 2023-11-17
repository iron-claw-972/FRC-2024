package frc.robot.commands.vision;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.GoToPose;
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
        addCommands(new GoToPose(()->getPose(gamePiece), drive));
    }
    /**
     * Gets the pose to move to, which is the game piece's pose with the angle between the robot and game piece
     * @param gamePiece The supplier for the game piece to move to
     * @return A Pose2d
     */
    public Pose2d getPose(Supplier<DetectedObject> gamePiece){
        return new Pose2d(
            gamePiece.get().pose.getX(),
            gamePiece.get().pose.getY(),
            new Rotation2d(gamePiece.get().getAngle())
        );
    }
}
