package frc.robot.subsystems.gpm;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Intake {
    
    public static CANSparkFlex motor1 = new CANSparkFlex(5, MotorType.kBrushless);
    public static double motorPower=-0.5;
    public static void setMotorPower(){
        motor1.set(motorPower);
    }
    public static void changeMoterSpeed(double speed){
        motorPower=speed;
    }
}