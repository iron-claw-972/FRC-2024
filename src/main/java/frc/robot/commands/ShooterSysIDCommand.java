// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.gpm.Shooter;
import frc.robot.util.SysId;

/** Add your docs here. */
public class ShooterSysIDCommand extends SequentialCommandGroup {
    private Config config = new Config();
    SysId sysId;
    public ShooterSysIDCommand(Shooter shooter){
        sysId = new SysId(
            "Shooter",
            x ->{
                    shooter.setVoltage(x);
                },
            x->{
                x.motor("left").angularPosition(shooter.getLeftShooterPosition());
                x.motor("left").angularVelocity(shooter.getLeftShooterSpeed());
                x.motor("left").voltage(shooter.getLeftShooterVoltage());
                x.motor("right").angularPosition(shooter.getRightShooterPosition());
                x.motor("right").angularVelocity(shooter.getRightShooterSpeed());
                x.motor("right").voltage(shooter.getRightShooterVoltage());
            },
            shooter,
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
