package frc.robot.subsystems.gpm_subsystem;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.constants.WristConstants;

public class Wrist extends PIDSubsystem{
    
    private CANSparkFlex wrist = new CANSparkFlex(WristConstants.MotorID, MotorType.kBrushless);

    private final SimpleMotorFeedforward wristFeedForward = 
    new SimpleMotorFeedforward(WristConstants.kS, WristConstants.kV); // use same ff for wrist?

    public Wrist() {
        super(new PIDController(WristConstants.kP, WristConstants.kI, WristConstants.kD));
        getController().setTolerance(WristConstants.kToleranceDegrees);
        getController().setSetpoint(WristConstants.kDegrees);
        wrist.setSecondaryCurrentLimit(0.5);
    }
    @Override
    public void useOutput(double output, double setpoint)  {
        wrist.setVoltage((-output-wristFeedForward.calculate(setpoint))/2);
        System.out.println("wrist" + wrist.getAppliedOutput());
    }
    @Override
    public double getMeasurement() {
        return wrist.getAbsoluteEncoder(Type.kDutyCycle).getPosition();
    }
    public boolean atSetpoint() {
        return getController().atSetpoint();
    }
}
