package frc.robot.commands.test_comm;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import frc.robot.constants.globalConst;
import lib.controllers.GameController;
import lib.controllers.GameController.Axis;

public class NeoTest {

   
  public CANSparkFlex motorShooterLeft = new CANSparkFlex(2, CANSparkFlex.MotorType.kBrushless);
  private static GameController driver = new GameController(globalConst.DRIVER_JOY);
  public CANSparkFlex motorShooterRight = new CANSparkFlex(1, CANSparkFlex.MotorType.kBrushless);




    // public NeoTest(Drivetrain drive) {
    //     m_drive = drive;
    //     addRequirements(drive);
    //   }

    //   @Override
      public void gogogo() {
        double speed2 = driver.get(Axis.RIGHT_Y);
        motorShooterLeft.set(speed2);
        motorShooterLeft.setIdleMode(IdleMode.kBrake);
        RelativeEncoder motorEncoderLeft = motorShooterLeft.getEncoder();

        motorShooterRight.set(-speed2*0.8);
        motorShooterRight.setIdleMode(IdleMode.kBrake);
        RelativeEncoder motorEncoderRight = motorShooterRight.getEncoder();

        // System.out.println("left: " + String.format("%09d", motorEncoderLeft.getVelocity()) + ", right: " + String.format("%09d", motorEncoderRight.getVelocity()));

        // System.out.println("running"+speed2);
        // System.out.println(motorShooterLeft.get());
      }
    
    }
