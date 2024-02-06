package frc.robot.subsystems.gpm_subsystem;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;

import java.io.FileWriter;

public class Outtake extends SubsystemBase {
    private CANSparkFlex bottomMotor = new CANSparkFlex(ShooterConstants.BOTTOM_MOTOR_ID, MotorType.kBrushless);
    private CANSparkFlex topMotor = new CANSparkFlex(ShooterConstants.TOP_MOTOR_ID, MotorType.kBrushless);
    private RelativeEncoder bottomMotorEncoder = bottomMotor.getEncoder();
    private RelativeEncoder topMotorEncoder = topMotor.getEncoder();
    private final PIDController topPID = new PIDController(.00005, 0, 0.00);
    private final PIDController bottomPID = new PIDController(.00005, 0, 0);

    private final SimpleMotorFeedforward shooterFF = new SimpleMotorFeedforward(0, 1.0/6000);
    // observe motor characteristics to get better FF values.

    FileWriter myWriter;
    public Outtake() {
        topPID.setTolerance(10); // rpm
        bottomPID.setTolerance(10);
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
    }

    public void setTargetRPM(double speed) {
        topPID.setSetpoint(speed*.5);
        bottomPID.setSetpoint(speed);
    }

    public void setTargetVelocity(double speed) {
        // convert speed to RPM
        // TODO: should double check that this is correct
        setTargetRPM(speed*60*100/(4*3.14*2.54));
    }

    public boolean atSetpoint() {
        return topPID.atSetpoint() && bottomPID.atSetpoint();
    }
}