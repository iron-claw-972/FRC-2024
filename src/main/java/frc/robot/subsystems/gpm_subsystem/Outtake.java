package frc.robot.subsystems.gpm_subsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.constants.ShooterConstants;
public class Outtake extends SubsystemBase {
    private CANSparkFlex bottomMotor = new CANSparkFlex(ShooterConstants.bottomMotorID, MotorType.kBrushless);
    private CANSparkFlex topMotor = new CANSparkFlex(ShooterConstants.topMotorID, MotorType.kBrushless);
    PIDController topPID = new PIDController(.05, 0, 0);
    PIDController bottomPID = new PIDController(.05, 0, 0);
    public final double targetrpm = 500;

    public Outtake() {
        topPID.setTolerance(10); // rpm
        bottomPID.setTolerance(10);
        topPID.setSetpoint(targetrpm);
        bottomPID.setSetpoint(targetrpm);
    }
    @Override
    public void periodic() {
        topMotor.set(topPID.calculate(topMotor.getAbsoluteEncoder(Type.kDutyCycle).getVelocity()));
        bottomMotor.set(bottomPID.calculate(bottomMotor.getAbsoluteEncoder(Type.kDutyCycle).getVelocity()));
    }

}
