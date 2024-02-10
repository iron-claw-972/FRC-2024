package frc.robot.subsystems.gpm_subsystem;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.OuttakeConstants;

public class Outtake extends SubsystemBase {
    private final CANSparkFlex bottomMotor = new CANSparkFlex(OuttakeConstants.BOTTOM_MOTOR_ID, MotorType.kBrushless);
    private final CANSparkFlex topMotor = new CANSparkFlex(OuttakeConstants.TOP_MOTOR_ID, MotorType.kBrushless);
    private final RelativeEncoder bottomMotorEncoder = bottomMotor.getEncoder();
    private final RelativeEncoder topMotorEncoder = topMotor.getEncoder();
    private final PIDController topPID = new PIDController(OuttakeConstants.TOP_P, OuttakeConstants.TOP_I,
            OuttakeConstants.TOP_D);
    private final PIDController bottomPID = new PIDController(OuttakeConstants.BOTTOM_P, OuttakeConstants.BOTTOM_I, OuttakeConstants.BOTTOM_D);

    // TODO: TUNE THIS
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(OuttakeConstants.S, OuttakeConstants.V);

    public Outtake() {
        topPID.setTolerance(OuttakeConstants.TOLERANCE);
        bottomPID.setTolerance(OuttakeConstants.TOLERANCE);
        bottomMotor.setInverted(true);
    }

    @Override
    public void periodic() {
        double topSpeed = topMotorEncoder.getVelocity();
        double bottomSpeed = bottomMotorEncoder.getVelocity();

        topMotor.set(topPID.calculate(topSpeed) + feedforward.calculate(topPID.getSetpoint()));
        bottomMotor.set(bottomPID.calculate(bottomSpeed) + feedforward.calculate(bottomPID.getSetpoint()));

        // TODO: Maybe add a method in ConversionUtils to convert from RPM to m/s and etc
        SmartDashboard.putNumber("top speed",topSpeed/60*4*3.14*2.54/100);
        SmartDashboard.putNumber("bottom speed",bottomSpeed/60*4*3.14*2.54/100);
        SmartDashboard.putBoolean("at setpoint?", atSetpoint());
        System.out.println(topSpeed/60*4*3.14*2.54/100+","+bottomSpeed/60*4*3.14*2.54/100);
    }

    public void setTargetRPM(double speed) {
        topPID.setSetpoint(speed*.5);
        bottomPID.setSetpoint(speed);
    }

    public void setTargetVelocity(double speed) {
        // convert speed to RPM
        setTargetRPM(speed*60*100/(4*3.14*2.54));
    }

    public boolean atSetpoint() {
        return topPID.atSetpoint() && bottomPID.atSetpoint();
    }
}