// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.subsystems.Drivetrain;

/** Add your docs here. */
public class SysID extends SequentialCommandGroup {
    
    Config config = new Config();
    Drivetrain drive;
    Mechanism mechanism;
    SysIdRoutine SysIdRoutine;
    
    public SysID(Drivetrain drive) {
        this.drive = drive;
        initializeMechanism();
        SysIdRoutine = new SysIdRoutine(config, mechanism);
        addCommands(
            new InstantCommand(()->drive.setStateDeadband(false)),
            new InstantCommand(()->drive.setOptimized(false)),
            new RunCommand(()->drive.setModuleStates((
            new SwerveModuleState[]{
            new SwerveModuleState(0,new Rotation2d()),
            new SwerveModuleState(0,new Rotation2d()),
            new SwerveModuleState(0,new Rotation2d()),
            new SwerveModuleState(0,new Rotation2d())}
        ),false)).withTimeout(1),new WaitCommand(1),
        SysIdRoutine.quasistatic(Direction.kForward));
    }
    
    
    
    public void initializeMechanism(){
        mechanism = new Mechanism(
            x->{drive.setModuleStates(new SwerveModuleState[]{new SwerveModuleState(0,new Rotation2d()),
                new SwerveModuleState(0,new Rotation2d()),
                new SwerveModuleState(0,new Rotation2d()),
                new SwerveModuleState(0,new Rotation2d())},true);
                drive.setDriveVoltages(x);
            },
            null,
            drive,
            "Drivetrain"
        );
    }
}
