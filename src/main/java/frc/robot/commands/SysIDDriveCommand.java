// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.SysId;

/** Add your docs here. */
public class SysIDDriveCommand extends SequentialCommandGroup {

    Drivetrain drive;
    Config config = new Config();
    SysId sysId;
    public SysIDDriveCommand(Drivetrain drive) {
        this.drive = drive;
        config = new Config(
            Units.Volts.of(0.5).per(Units.Seconds.of(1)),
            Units.Volts.of(7),
            Units.Seconds.of(20),
            null
        );
        TalonFX[] driveMotors = {
            drive.getModules()[0].getDriveMotor(),
            drive.getModules()[1].getDriveMotor(),
            drive.getModules()[2].getDriveMotor(),
            drive.getModules()[3].getDriveMotor()
        };
        TalonFX[] angleMotors = {
            drive.getModules()[0].getAngleMotor(),
            drive.getModules()[1].getAngleMotor(),
            drive.getModules()[2].getAngleMotor(),
            drive.getModules()[3].getAngleMotor()
        };
        Rotation2d[] angles = {
            Rotation2d.fromDegrees(-45-180),
            Rotation2d.fromDegrees(45),
            Rotation2d.fromDegrees(45+180),
            Rotation2d.fromDegrees(-45),
        };
        sysId = new SysId(
            "Drivetrain",
            driveMotors,
            angleMotors,
            drive,
            config,
            angles
        );
        addCommands(
            sysId.runQuasisStatic(Direction.kForward)
            );
    }

    

}
