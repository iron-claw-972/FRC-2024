package frc.robot.subsystems.gpm_subsystem;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.constants.ShooterConstants;

public class Outtake extends PIDSubsystem {
    private final CANSparkFlex bottomMotor = new CANSparkFlex(ShooterConstants.BOTTOM_MOTOR_ID, MotorType.kBrushless);
    private final CANSparkFlex topMotor = new CANSparkFlex(ShooterConstants.TOP_MOTOR_ID, MotorType.kBrushless);


    private final SimpleMotorFeedforward shooterFeedForward = 
    new SimpleMotorFeedforward(ShooterConstants.S, ShooterConstants.V); // use same ff for wrist?

    public Outtake() {
        super(new PIDController(ShooterConstants.P, ShooterConstants.I, ShooterConstants.D));
        getController().setTolerance(ShooterConstants.TOLERANCE_RPM);
        getController().setSetpoint(ShooterConstants.TARGET_RPM);
        topMotor.setSecondaryCurrentLimit(0.5);
        bottomMotor.setSecondaryCurrentLimit(0.5);
    }

    @Override
    public void useOutput(double output, double setpoint)  {
        bottomMotor.setVoltage((-output - shooterFeedForward.calculate(setpoint)) / 2);
        topMotor.setVoltage((output + shooterFeedForward.calculate(setpoint)) / 2);
        System.out.println("b" + bottomMotor.getAppliedOutput());
        System.out.println("t" + topMotor.getAppliedOutput());
    }

    @Override
    public double getMeasurement()  {
        return (topMotor.getAbsoluteEncoder(Type.kDutyCycle).getVelocity() + bottomMotor.getAbsoluteEncoder(Type.kDutyCycle).getVelocity()) / 2;
    }

    public boolean atSetpoint() {
        return getController().atSetpoint();
    }
}