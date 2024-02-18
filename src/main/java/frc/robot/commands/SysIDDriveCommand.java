// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.constants.swerve.DriveConstants;
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
            Units.Volts.of(0.2).per(Units.Seconds.of(1)),
            Units.Volts.of(3),
            Units.Seconds.of(5),
            (x)->SignalLogger.writeString("state", x.toString())
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
            Rotation2d.fromDegrees(0),//-45-180
            Rotation2d.fromDegrees(0),//45
            Rotation2d.fromDegrees(0),//45+180
            Rotation2d.fromDegrees(0),//-45
        };
        sysId = new SysId(
            "Drivetrain",
            x ->{
                    for (int i =0; i<angleMotors.length;i++){ 
                        angleMotors[i].setControl(new PositionDutyCycle(angles[i].getRotations()*DriveConstants.kModuleConstants.angleGearRatio));
                    }
                
                    for (TalonFX motor: driveMotors){
                        motor.setVoltage(x.magnitude());
                    }
                    
                },
            drive,
            config
        );
        addCommands(
            sysId.runQuasisStatic(Direction.kForward),
            new WaitCommand(0.5),
            sysId.runQuasisStatic(Direction.kReverse),
            new WaitCommand(0.5),
            sysId.runDynamic(Direction.kForward),
            new WaitCommand(0.5),
            sysId.runDynamic(Direction.kReverse)
            );
    }

    

}
