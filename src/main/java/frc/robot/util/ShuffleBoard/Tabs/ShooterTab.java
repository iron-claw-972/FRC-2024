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
    private GenericEntry P;
    private GenericEntry I;
    private GenericEntry D;
    private GenericEntry P2;
    private GenericEntry I2;
    private GenericEntry D2;

    Shooter shooter;
    public ShooterTab(Shooter shooter){
        this.shooter = shooter;
    }

    public void createEntries(){
        tab = Shuffleboard.getTab("Shooter");
        shooterLeftSpeed = tab.add("Shooter Left Speed",0).getEntry();
        ShooterLeftExpectedSpeed = tab.add("Shooter Left Expected Speed",0).getEntry();
        shooterRightSpeed = tab.add("Shooter Right Speed",0).getEntry();
        ShooterRightExpectedSpeed = tab.add("Shooter Right Expected Speed",0).getEntry();
        P = tab.add("right p",0).getEntry();
        I = tab.add("right i",0).getEntry();
        D = tab.add("right d",0).getEntry();
        P2 = tab.add("left p",0).getEntry();
        I2 = tab.add("left i",0).getEntry();
        D2 = tab.add("left d",0).getEntry();
    }

    public void update(){
        shooterLeftSpeed.setDouble(shooter.getLeftMotorRPM());
        ShooterLeftExpectedSpeed.setDouble(shooter.getLeftSetpoint());
        shooterRightSpeed.setDouble(shooter.getRightMotorRPM());
        ShooterRightExpectedSpeed.setDouble(shooter.getRightSetpoint());
        shooter.setLeftp(P2.getDouble(0));
        shooter.setLefti(I2.getDouble(0));
        shooter.setLeftd(D2.getDouble(0));
        shooter.setRightp(P.getDouble(0));
        shooter.setRighti(I.getDouble(0));
        shooter.setRightd(D.getDouble(0));
    }
}
