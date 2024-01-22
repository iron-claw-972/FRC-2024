package frc.robot.subsystems.gpm_subsystem;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.constants.ShooterConstants;

public class Shooter extends PIDSubsystem{
    private CANSparkFlex bottomMotor = new CANSparkFlex(ShooterConstants.bottomMotorID, MotorType.kBrushless);
    private CANSparkFlex topMotor = new CANSparkFlex(ShooterConstants.topMotorID, MotorType.kBrushless);

    private final SimpleMotorFeedforward shooterFeedForward = 
    new SimpleMotorFeedforward(ShooterConstants.kS, ShooterConstants.kV);

    public Shooter() {
        super(new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD));
        getController().setTolerance(ShooterConstants.kShooterToleranceRPM);
        getController().setSetpoint(ShooterConstants.kShooterTargetRPM);
    }

    @Override
    public void useOutput(double output, double setpoint)  {
        bottomMotor.setVoltage(output + shooterFeedForward.calculate(setpoint));
        topMotor.setVoltage(output + shooterFeedForward.calculate(setpoint));
    }

    @Override
    public double getMeasurement()  {
        return topMotor.get() + bottomMotor.get() / 2;
    }

    public boolean atSetpoint() {
        return getController().atSetpoint();
    }
}


