// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.ShuffleBoard;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.subsystems.Drivetrain;

/** Add your docs here. */
public class Field {
    Drivetrain drive;
    Field2d feild = new Field2d();
    Pose2d chassisPose;
    Pose2d[] modulePositions = new Pose2d[4];
    
    public Field(Drivetrain drive){
        this.drive = drive;
        Shuffleboard.getTab("Swerve").add(feild);
    } 
    
    public void updateModulePositions(){
        if (chassisPose != null){
            chassisPose = drive.getPose();
        }
        else{
            chassisPose = new Pose2d();
        }
        
        for (int i = 0; i<4; i++ ){
            Translation2d postion = DriveConstants.swerveModuleLocations[i].
            rotateBy(chassisPose.getRotation())
            .plus(chassisPose.getTranslation());

            Rotation2d moduleRotation = drive.getModulePositions()[i].angle
            .plus(chassisPose.getRotation());

            modulePositions[i] = new Pose2d(postion, moduleRotation);
            if (drive.getModules()[i].getState().speedMetersPerSecond<0){
                modulePositions[i].getRotation().plus(Rotation2d.fromDegrees(180));
            }
        }
    }

    public void updateFeild(){
        updateModulePositions();
        feild.setRobotPose(chassisPose);
        feild.getObject("modules").setPoses(modulePositions);      
    }

}
