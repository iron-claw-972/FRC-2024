package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Robot;
import frc.robot.commands.Climb.Chain;
import frc.robot.constants.miscConstants.VisionConstants.STAGE_POSES;
import frc.robot.subsystems.Drivetrain;

public class ChainAlign extends GoToPose {
    public ChainAlign(Chain chain, Drivetrain drive){
        super(()->getChainPose(chain), drive);
    }
    private static Pose2d getChainPose(Chain chain){
        switch(chain){
            case LEFT:
                return Robot.getAlliance()==Alliance.Red?STAGE_POSES.RED_LEFT.climb:STAGE_POSES.BLUE_LEFT.climb;
            case CENTER:
                return Robot.getAlliance()==Alliance.Red?STAGE_POSES.RED_CENTER.climb:STAGE_POSES.BLUE_CENTER.climb;
            case RIGHT:
                return Robot.getAlliance()==Alliance.Red?STAGE_POSES.RED_RIGHT.climb:STAGE_POSES.BLUE_RIGHT.climb;
            default:
                return null;
        }
    }
}
