package frc.robot.commands;

import java.sql.Driver;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import lib.controllers.GameController;
import lib.controllers.GameController.Axis;
import lib.controllers.PS5Controller.PS5Axis;
import lib.controllers.PS5Controller;

public class MotorTest {

   
  public TalonFX motor1 = new TalonFX(52),
  motor2 = new TalonFX(58),
  motor3 = new TalonFX(61);
  private final PS5Controller kDriver = new PS5Controller(Constants.DRIVER_JOY);


    // public NeoTest(Drivetrain drive) {
    //     m_drive = drive;
    //     addRequirements(drive);
    //   }

    //   @Override
      public void gogogo() {
        double speed2 = kDriver.get(PS5Axis.RIGHT_Y);

        motor1.set(speed2);

        motor2.set(speed2);

        motor3.set(speed2);
        // System.out.println(motorShooterLeft.get());
      }
    
    }