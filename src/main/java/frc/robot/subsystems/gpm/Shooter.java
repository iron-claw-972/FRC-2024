package frc.robot.subsystems.gpm;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    // constants for Pids
    public static final double TOP_P = 0.00005;
    public static final double TOP_I = 0;
    public static final double TOP_D = 0;

    public static final double S = 0;
    public static final double V = 1.0/6000;

    public static final double BOTTOM_P = 0.00005;
    public static final double BOTTOM_I = 0;
    public static final double BOTTOM_D = 0;

    /**
     * In RPM
     */
    public static final double TOLERANCE = 10;

    private final CANSparkFlex bottomMotor = new CANSparkFlex(ShooterConstants.BOTTOM_MOTOR_ID, MotorType.kBrushless);
    private final CANSparkFlex topMotor = new CANSparkFlex(ShooterConstants.TOP_MOTOR_ID, MotorType.kBrushless);
    private final RelativeEncoder bottomMotorEncoder = bottomMotor.getEncoder();
    private final RelativeEncoder topMotorEncoder = topMotor.getEncoder();
    private final PIDController topPID = new PIDController(TOP_P, TOP_I, TOP_D);
    private final PIDController bottomPID = new PIDController(BOTTOM_P, BOTTOM_I, BOTTOM_D);

    // TODO: TUNE THIS
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(S, V);

    public Shooter() {
        topPID.setTolerance(TOLERANCE);
        bottomPID.setTolerance(TOLERANCE);
        bottomMotor.setInverted(true);
    }

    @Override
    public void periodic() {
        double topSpeed = topMotorEncoder.getVelocity();
        double bottomSpeed = bottomMotorEncoder.getVelocity();

        topMotor.set(topPID.calculate(topSpeed) + feedforward.calculate(topPID.getSetpoint()));
        bottomMotor.set(bottomPID.calculate(bottomSpeed) + feedforward.calculate(bottomPID.getSetpoint()));

        SmartDashboard.putNumber("top speed", shooterRPMToSpeed(topSpeed));
        SmartDashboard.putNumber("bottom speed", shooterRPMToSpeed(bottomSpeed));
        SmartDashboard.putBoolean("at setpoint?", atSetpoint());
        System.out.println(shooterRPMToSpeed(topSpeed) + "," + shooterRPMToSpeed(bottomSpeed));
    }

	/**
	* Converts an RPM to a speed at the surface of the shooter wheels.
	*
	* @param  rpm the RPM to convert
	* @return     the converted value in m/s
	* @see        shooterSpeedToRPM
	*/
	public static double shooterRPMToSpeed(double rpm) {
			return (rpm / 60) * (4 * Math.PI * 0.0254);
	}

	/**
	* Converts a speed at the surface of the shooter wheels to an RPM value.
	*
	* @param  speed the speed to convert in m/s
	* @return       the converted value in RPM
	* @see          shooterRPMToSpeed
	*/
	public static double shooterSpeedToRPM(double speed) {
			return (speed * 60) / (4 * Math.PI * 0.0254);
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
        setTargetRPM(shooterSpeedToRPM(speed));
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
	* Gets the RPM of the top motor via the encoder's getVelocity method.
	*
	* @return the top motor's RPM
	* @see    getBottomMotorRPM
	* @see    getTopMotorSpeed
	* @see    atSetpoint
	*/
	public double getTopMotorRPM() {
		return topMotorEncoder.getVelocity();
	}

	/**
	* Gets the RPM of the bottom motor via the encoder's getVelocity method.
	*
	* @return the bottom motor's RPM
	* @see    getTopMotorRPM
	* @see    getBottomMotorSpeed
	* @see    atSetpoint
	*/
	public double getBottomMotorRPM() {
		return bottomMotorEncoder.getVelocity();
	}

	/**
	* Gets the speed at the surface of the top wheels.
	*
	* @return the top wheel's speed in m/s
	* @see    getBottomMotorSpeed
	* @see    getTopMotorRPM
	* @see    atSetpoint
	*/
	public double getTopMotorSpeed() {
		return shooterRPMToSpeed(getTopMotorRPM());
	}

	/**
	* Gets the speed at the surface of the bottom wheels.
	*
	* @return the bottom wheel's speed in m/s
	* @see    getTopMotorSpeed
	* @see    getBottomMotorRPM
	* @see    atSetpoint
	*/
	public double getBottomMotorSpeed() {
		return shooterRPMToSpeed(getBottomMotorRPM());
	}
	
	/**
	* Gets the difference in RPM between the top motor and bottom motor.
	* 
	* @return the top motor's RPM - the bottom motor-s RPM
	* @see    getMotorSpeedDifference
	* @see    getTopMotorRPM
	* @see    getBottomMotorRPM
	* @see    atSetpoint
	*/
	public double getMotorRPMDifference() {
		return getTopMotorRPM() - getBottomMotorRPM();
	}
	
	/**
	* Gets the difference in speed at the circumference of the motor's wheels
	* between the top motor and bottom motor.
	* 
	* @return the top motor's speed - the bottom motor's speed
	* @see    getMotorRPMDifference
	* @see    getTopMotorSpeed
	* @see    getBottomMotorSpeed
	* @see    atSetpoint
	*/
	public double getMotorSpeedDifference() {
		return getTopMotorSpeed() - getBottomMotorSpeed();
	}
}
