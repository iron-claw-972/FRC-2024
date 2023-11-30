package frc.robot.commands.vision;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.GoToPose;
import frc.robot.constants.swerve.DriveConstants;
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
     * @param gamePieceSupplier The supplier for the game piece to move to
     * @return A Pose2d
     */
    public Pose2d getPose(Supplier<DetectedObject> gamePieceSupplier){
        double dist = DriveConstants.kRobotWidthWithBumpers/2;
        DetectedObject gamePiece = gamePieceSupplier.get();
        // If the game piece does not exist or is inside the robot, do nothing
        if(gamePiece==null || gamePiece.getDistance()<dist){
            return null;
        }
        double angle = gamePiece.getAngle();
        return new Pose2d(
            gamePiece.pose.getX()-dist*Math.cos(angle),
            gamePiece.pose.getY()-dist*Math.sin(angle),
            new Rotation2d(angle)
        );
    }
}
