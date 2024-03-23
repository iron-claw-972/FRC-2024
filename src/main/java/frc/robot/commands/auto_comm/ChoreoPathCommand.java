package frc.robot.commands.auto_comm;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.SupplierCommand;
import frc.robot.subsystems.Drivetrain;

public class ChoreoPathCommand extends SequentialCommandGroup {
    private final Drivetrain drive;
    private final PathPlannerPath path;

    public ChoreoPathCommand(String pathName, boolean resetOdemetry, Drivetrain drive){
        this.drive = drive;
        this.path = PathPlannerPath.fromChoreoTrajectory(pathName);
        addCommands(
                new InstantCommand(()->resetOdemetry(resetOdemetry)),
                new SupplierCommand(()-> AutoBuilder.followPath(path), drive)
        );
    }

    public void resetOdemetry(boolean resetOdemetry){
        if (resetOdemetry){
            if(RobotContainer.getAllianceColorBooleanSupplier().getAsBoolean()){
                drive.resetOdometry(path.flipPath().getPreviewStartingHolonomicPose());
            }
            else{
                drive.resetOdometry(path.getPreviewStartingHolonomicPose());
            }
        }
    }
}

