package frc.robot.subsystems.gpm_subsystem;
import java.io.FileWriter;   // Import the FileWriter class
import java.io.IOException;   // Import the FileWriter class
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.ShooterConstants;

public class Outtake extends SubsystemBase {
    private CANSparkFlex bottomMotor = new CANSparkFlex(ShooterConstants.bottomMotorID, MotorType.kBrushless);
    private CANSparkFlex topMotor = new CANSparkFlex(ShooterConstants.topMotorID, MotorType.kBrushless);
    private RelativeEncoder bottomMotorEncoder = bottomMotor.getEncoder();
    private RelativeEncoder topMotorEncoder = topMotor.getEncoder();
    PIDController topPID = new PIDController(.00005, 0, 0.00);
    PIDController bottomPID = new PIDController(.00005, 0, 0);
    
    SimpleMotorFeedforward shooterFF = new SimpleMotorFeedforward(0, 1.0/6000);
    // observe motor characteristics to get better FF values.

    FileWriter myWriter;
    public Outtake() {
        topPID.setTolerance(10); // rpm
        bottomPID.setTolerance(10);

        SmartDashboard.putData("speed 500", new InstantCommand(() -> setTargetRPM(500)));
        SmartDashboard.putData("speed 1500", new InstantCommand(() -> setTargetRPM(1500)));
        SmartDashboard.putData("speed 2000", new InstantCommand(() -> setTargetRPM(2000)));
        SmartDashboard.putData("speed 2500", new InstantCommand(() -> setTargetRPM(2500)));
        SmartDashboard.putData("speed 3000", new InstantCommand(() -> setTargetRPM(3000)));
        SmartDashboard.putData("speed 3500", new InstantCommand(() -> setTargetRPM(3500)));
        SmartDashboard.putData("reset", new InstantCommand(() -> setTargetRPM(0)));
        bottomMotor.setInverted(true);

    }

    @Override
    public void periodic() {
        double topspeed = topMotorEncoder.getVelocity();
        double bottomspeed = bottomMotorEncoder.getVelocity();

        topMotor.set(topPID.calculate(topspeed) + shooterFF.calculate(topPID.getSetpoint()));
        bottomMotor.set(bottomPID.calculate(bottomspeed) + shooterFF.calculate(bottomPID.getSetpoint()));

        SmartDashboard.putNumber("top speed",topspeed/60*4*3.14*2.54/100);
        SmartDashboard.putNumber("bottom speed",bottomspeed/60*4*3.14*2.54/100);
        SmartDashboard.putBoolean("at setpoint?", atSetpoint());
        System.out.println(topspeed/60*4*3.14*2.54/100+","+bottomspeed/60*4*3.14*2.54/100);
        /*
        try {
     myWriter= new FileWriter("shooter_speed.txt");
        myWriter.write(topspeed+","+bottomspeed+"\n");
        myWriter.close();
        } catch (IOException e) {
            System.out.println("this");
        }
        */
    }

    public void setTargetRPM(double speed) {
        topPID.setSetpoint(speed*.5);
        bottomPID.setSetpoint(speed);
        
    }

    public boolean atSetpoint() {
        return topPID.atSetpoint() && bottomPID.atSetpoint();
    }
}
