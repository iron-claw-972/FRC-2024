// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.ShuffleBoard.Tabs;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.subsystems.gpm.Shooter;
import frc.robot.util.ShuffleBoard.ShuffleBoardTabs;

/** Add your docs here. */
public class ShooterTab extends ShuffleBoardTabs {
    private GenericEntry shooterLeftSpeed;
    private GenericEntry ShooterLeftExpectedSpeed;
    private GenericEntry shooterRightSpeed;
    private GenericEntry ShooterRightExpectedSpeed;
    Shooter shooter;
    public ShooterTab(Shooter shooter){
        this.shooter = shooter;
    }

    public void createEntries(){
        tab = Shuffleboard.getTab("Shooter");

        shooterLeftSpeed = tab.add("Shooter Speed",0).getEntry();
        ShooterLeftExpectedSpeed = tab.add("Shooter Expected Speed",0).getEntry();
        shooterRightSpeed = tab.add("Shooter Speed",0).getEntry();
        ShooterRightExpectedSpeed = tab.add("Shooter Expected Speed",0).getEntry();
    }

    public void update(){
        shooterLeftSpeed.setDouble(shooter.getLeftMotorRPM());
        ShooterLeftExpectedSpeed.setDouble(shooter.getLeftSetpoint());
        shooterRightSpeed.setDouble(shooter.getRightMotorRPM());
        ShooterRightExpectedSpeed.setDouble(shooter.getRightSetpoint());
    }
}
