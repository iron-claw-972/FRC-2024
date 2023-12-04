package frc.robot.commands.drive_comm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drivetrain;

public class ResetYaw extends CommandBase{
    
        Drivetrain m_drive;
        boolean backwards;

    public ResetYaw(boolean backwards, Drivetrain drive)  {
         m_drive = drive;
        this.backwards = backwards;
    }
    public void initialize() {
        if(backwards)  {
            m_drive.setYaw(new Rotation2d((DriverStation.getAlliance() == Alliance.Blue) ? 0 : Math.PI));
        }
        
        else  {
            m_drive.setYaw(new Rotation2d((DriverStation.getAlliance() == Alliance.Red) ? 0 : Math.PI));
        }
    }

    public void execute()  {

    }

    public boolean isFinished()  {
        return true;
    }

    public void end(boolean terminated)  {

    }
}   
