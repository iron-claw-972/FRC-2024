package frc.robot.subsystems.gpm;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    // constants for Pids
    private static final double TOP_P = 0.00005;
    private static final double TOP_I = 0;
    private static final double TOP_D = 0;

    private static final double S = 0;
    private static final double V = 1.0/6000;

    private static final double BOTTOM_P = 0.00005;
    private static final double BOTTOM_I = 0;
    private static final double BOTTOM_D = 0;

    /**
     * In RPM
     */
    private static final double TOLERANCE = 10;

	private static final double massColson = 0.245;
	private static final double radiusColson = 2.0 * 0.0254;
	private static final double moiColson = 0.5 * massColson * radiusColson * radiusColson;
	private static final double moiShaft = moiColson * 4;

    private final CANSparkFlex topMotor = new CANSparkFlex(ShooterConstants.TOP_MOTOR_ID, MotorType.kBrushless);
    private final RelativeEncoder topMotorEncoder = topMotor.getEncoder();
    private final PIDController topPID = new PIDController(TOP_P, TOP_I, TOP_D);
	private FlywheelSim topFlywheelSim;
	private double topMotorSpeedSim;

    private final CANSparkFlex bottomMotor = new CANSparkFlex(ShooterConstants.BOTTOM_MOTOR_ID, MotorType.kBrushless);
    private final RelativeEncoder bottomMotorEncoder = bottomMotor.getEncoder();
    private final PIDController bottomPID = new PIDController(BOTTOM_P, BOTTOM_I, BOTTOM_D);
	private FlywheelSim bottomFlywheelSim;
	private double bottomMotorSpeedSim;

    // TODO: TUNE THIS
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(S, V);

    public Shooter() {
        topPID.setTolerance(TOLERANCE);
        bottomPID.setTolerance(TOLERANCE);
        bottomMotor.setInverted(true);

		if (RobotBase.isSimulation()) {
			topFlywheelSim = new FlywheelSim(
				DCMotor.getNeoVortex(1),
				1.0,
				moiShaft);
			bottomFlywheelSim = new FlywheelSim(
				DCMotor.getNeoVortex(1),
				1.0,
				moiShaft);
		}
    }

    @Override
    public void periodic() {
        double topSpeed = getTopMotorSpeed();
        double bottomSpeed = getBottomMotorSpeed();

        topMotor.set(topPID.calculate(topSpeed) + feedforward.calculate(topPID.getSetpoint()));
        bottomMotor.set(bottomPID.calculate(bottomSpeed) + feedforward.calculate(bottomPID.getSetpoint()));

        SmartDashboard.putNumber("top speed", shooterRPMToSpeed(topSpeed));
        SmartDashboard.putNumber("bottom speed", shooterRPMToSpeed(bottomSpeed));
        SmartDashboard.putBoolean("at setpoint?", atSetpoint());
		// do not print stuff out; look on the SmartDashboard
        // System.out.println(ConversionUtils.shooterRPMToSpeed(topSpeed) + "," + ConversionUtils.shooterRPMToSpeed(bottomSpeed));
    }

	@Override
	public void simulationPeriodic() {
		double voltage = 12.0;

		double topPower = topMotor.get();
		double bottomPower = bottomMotor.get();
		
		topFlywheelSim.setInputVoltage(topPower * voltage);
		bottomFlywheelSim.setInputVoltage(bottomPower * voltage);

		topFlywheelSim.update(0.020);
		bottomFlywheelSim.update(0.020);

		topMotorSpeedSim += topFlywheelSim.getAngularVelocityRPM();
		bottomMotorSpeedSim += bottomFlywheelSim.getAngularVelocityRPM();
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
	*/
	public void setTargetRPM(double speedTop, double speedBottom) {
		topPID.setSetpoint(speedTop);
		bottomPID.setSetpoint(speedBottom);
	}

	/**
	* Sets the RPM both shooter motors try to spin up to.
	*
	* @param speed speed to spin to in RPM
	* @see         setTargetVelocity
	*/
    public void setTargetRPM(double speed) {
        setTargetRPM(speed * 0.5, speed);
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
	* Gets the RPM of the top motor, checking whether this is a simulation.
	*
	* @return the top motor's RPM
	* @see    getBottomMotorRPM
	* @see    getTopMotorSpeed
	* @see    atSetpoint
	*/
	public double getTopMotorRPM() {
		if (RobotBase.isSimulation()) {
			return topMotorEncoder.getVelocity();
		} else {
			return topMotorSpeedSim;
		}
	}

	/**
	* Gets the RPM of the bottom motor, checking whether this is a simulation.
	*
	* @return the bottom motor's RPM
	* @see    getTopMotorRPM
	* @see    getBottomMotorSpeed
	* @see    atSetpoint
	*/
	public double getBottomMotorRPM() {
		if (RobotBase.isSimulation()) {
			return bottomMotorEncoder.getVelocity();
		} else {
			return bottomMotorSpeedSim;
		}
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
