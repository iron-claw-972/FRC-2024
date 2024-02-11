package frc.robot.subsystems.gpm;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;
import frc.robot.util.ConversionUtils;

public class Shooter extends SubsystemBase {
    private final CANSparkFlex bottomMotor = new CANSparkFlex(ShooterConstants.BOTTOM_MOTOR_ID, MotorType.kBrushless);
    private final CANSparkFlex topMotor = new CANSparkFlex(ShooterConstants.TOP_MOTOR_ID, MotorType.kBrushless);
    private final RelativeEncoder bottomMotorEncoder = bottomMotor.getEncoder();
    private final RelativeEncoder topMotorEncoder = topMotor.getEncoder();
    private final PIDController topPID = new PIDController(ShooterConstants.TOP_P, ShooterConstants.TOP_I,
            ShooterConstants.TOP_D);
    private final PIDController bottomPID = new PIDController(ShooterConstants.BOTTOM_P, ShooterConstants.BOTTOM_I, ShooterConstants.BOTTOM_D);

    // TODO: TUNE THIS
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(ShooterConstants.S, ShooterConstants.V);

    public Shooter() {
        topPID.setTolerance(ShooterConstants.TOLERANCE);
        bottomPID.setTolerance(ShooterConstants.TOLERANCE);
        bottomMotor.setInverted(true);
    }

    @Override
    public void periodic() {
        double topSpeed = topMotorEncoder.getVelocity();
        double bottomSpeed = bottomMotorEncoder.getVelocity();

        topMotor.set(topPID.calculate(topSpeed) + feedforward.calculate(topPID.getSetpoint()));
        bottomMotor.set(bottomPID.calculate(bottomSpeed) + feedforward.calculate(bottomPID.getSetpoint()));

        SmartDashboard.putNumber("top speed", ConversionUtils.shooterRPMToSpeed(topSpeed));
        SmartDashboard.putNumber("bottom speed", ConversionUtils.shooterRPMToSpeed(bottomSpeed));
        SmartDashboard.putBoolean("at setpoint?", atSetpoint());
		// do not print stuff out; look on the SmartDashboard
        // System.out.println(ConversionUtils.shooterRPMToSpeed(topSpeed) + "," + ConversionUtils.shooterRPMToSpeed(bottomSpeed));
    }

	/**
	* Sets the speed both shooter motors try to spin up to independantly.
	*
	* @param speedTop    the speed tho top motor will spin to in RPM
	* @param speedBottom the speed the bottom motor will spin to in RPM
	* @see               setTargetRPM
	*/
	public void setBothTargetRPMs(double speedTop, double speedBottom) {
		topPID.setSetpoint(speedTop);
		bottomPID.setSetpoint(speedBottom);
	}

	/**
	* Sets the RPM both shooter motors try to spin up to.
	*
	* @param speed speed to spin to in RPM
	* @see         setBothTargetRPMs
	* @see         setTargetVelocity
	*/
    public void setTargetRPM(double speed) {
        setBothTargetRPMs(speed * 0.5, speed);
    }

	/**
	* Set the target velocity of a note exiting the shooter.
	*
	* @param speed speed to spin to in m/s
	* @see         setTargetRPM
	*/
    public void setTargetVelocity(double speed) {
        // convert speed to RPM
        setTargetRPM(ConversionUtils.shooterSpeedToRPM(speed));
    }

	/**
	* Checks whether both motor PIDs are at their setpoints.
	*
	* @return boolean indicating whether both PIDs are at their setpoints
	*/
    public boolean atSetpoint() {
        return topPID.atSetpoint() && bottomPID.atSetpoint();
    }

	/**
	* Gets the RPMs of both motors.
	*
	* @return a list containing [top motor RPM, bottom motor RPM]
	* @see    getMotorVelocities
	* @see    atSetpoint
	*/
	public double[] getMotorRPMs() {
		return new double[] {topMotorEncoder.getVelocity(), bottomMotorEncoder.getVelocity()};
	}

	/**
	* Gets the velocities of both motors.
	*
	* @return an array containing [top motor velocity, bottom motor velocity]
	* @see    getMotorRPMs
	* @see    atSetpoint
	*/
	public double[] getMotorVelocities() {
		double[] rpms = getMotorRPMs();
		return new double[] {ConversionUtils.shooterRPMToSpeed(rpms[0]), ConversionUtils.shooterRPMToSpeed(rpms[1])};
	}
}
