package frc.robot.subsystems.gpm;

import java.time.Duration;
import java.util.ArrayList;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.Constants;
import frc.robot.util.LogManager;

/**
 * Subsystem that controls the arm.
 * <p>
 * The indexer and shooter are at the end of the arm.
 */
public class Arm extends SubsystemBase {

    /**
     * The motors are stored in an array.
     * <p>
     * The first entry is the master motor.
     * <p>
     * The first set of entries are the left side motors, and the second set are the right side.
     */
    private final TalonFX[] motors = new TalonFX[ArmConstants.MOTOR_IDS.length];
    // DCMotor model is 4 Kraken X60
    protected static final DCMotor motorModel = DCMotor.getKrakenX60(ArmConstants.MOTOR_IDS.length);

    /**
     * Gearbox ratio
     * <p>
     * The gearing starting from the motor is:
     * <p>
     * 8T : 72T
     * <p>
     * 18T : 84T
     * <p>
     * 16T : 64T
     */
    protected static final double GEARING = (72.0 / 8.0) * (84.0 / 18.0) * (64.0 / 16.0);

    // Encoder in the TalonFX.... currently not used.
    //   check docs. Normal update rate is 4 Hz.
    StatusSignal<Double> rotorPositionSignal;

    // DutyCycle control
    // TODO: change to voltage control
    private final DutyCycleOut m_request = new DutyCycleOut(0);
    public double dutyCycle = 0;

    /**
     * REV absolute encoder.
     * <p>
     *  It is mounted on the right hand side. CCW rotation is arm raises.
     */
    private final DutyCycleEncoder encoder = new DutyCycleEncoder(ArmConstants.ENCODER_ID);
    /** this instance sets the REV absolute encoder value during simulations */
    private DutyCycleEncoderSim encoderSim;
    /** 
     * REV encoder offset in radians.
     * <p>
     * WARNING: This value will change if the belt driving the REV encoder slips!
     * New value for OFFSET
     * stow is 0.599
     * high is 0.357
     */
    protected static final double OFFSET = 0.59 + Units.radiansToRotations(ArmConstants.MIN_ANGLE_RADS);
    /** REV encoder scale factor. This is fixed. */
    protected static final double DISTANCE_PER_ROTATION = -2 * Math.PI;

    // Motor PID control
    private static final double TOLERANCE = Units.degreesToRadians(6.0);
    // P = 5 worked during simulation
    private static final double P = 0.6;
    private static final double I = 0;
    private static final double D = 0;
    private final PIDController pid = new PIDController(P, I, D);

    // Motor feedforward control
    public static final double S = 0;
    public static final double G = 0;
    public static final double V = 0;
    private final ArmFeedforward feedforward = new ArmFeedforward(S, G, V);

    /** arm simulator */
    private SingleJointedArmSim simulation;

    // Mechanism display.
    // The display is always built, but it only changes during simulation.
    private Mechanism2d wristDisplay = new Mechanism2d(90, 90);
    private MechanismRoot2d pivot = wristDisplay.getRoot("ArmPivot", 45, 45);
    private MechanismLigament2d moving = pivot.append(
        new MechanismLigament2d(
            "Moving",
            30,
            Units.radiansToDegrees(ArmConstants.START_ANGLE_RADS),
            6,
            new Color8Bit(Color.kYellow)));

    public Arm() {
        // set the PID tolerance
        pid.setTolerance(TOLERANCE);

        // set the PID the starting position
        pid.setSetpoint(ArmConstants.START_ANGLE_RADS);

		// set the arm offset
		// assumes the arm is at its lowest position on startup
		// if the arm is not at its stowedSetpoint, then this will likely break the arm
		//OFFSET = getAngleRad() - ArmConstants.MIN_ANGLE_RADS;

        // make the encoder report arm angle in radians
        encoder.setDistancePerRotation(DISTANCE_PER_ROTATION);
        encoder.setPositionOffset(OFFSET);

        // consider each of the motors
        for (int i = 0; i < motors.length; i++) {
            // create the motor
            motors[i] = new TalonFX(ArmConstants.MOTOR_IDS[i]);
            // TODO: why is this done here? The slave should follow when neutral mode is set below.
            motors[i].setNeutralMode(NeutralModeValue.Brake);

            // i==0 is the master; the others are slaves
            if (i > 0) {
                // set slave mode
                // the master is MOTOR_IDS[0]
                // invert master for ids 2 and 3
                motors[i].setControl(new Follower(ArmConstants.MOTOR_IDS[0], (i >= motors.length / 2)));
            }
        }

        // common configuration for each motor
        // configure the master after the slaves have been linked so slaves will copy the same settings.
        motors[0].setNeutralMode(NeutralModeValue.Brake);
        motors[0].setInverted(false);
        motors[0].getConfigurator().apply(ArmConstants.currentConfig);

        // Phoenix v6 rotor position signal
        rotorPositionSignal = motors[0].getRotorPosition();

        // possibly set up simulations
        if (RobotBase.isSimulation()) {
            simulation = new SingleJointedArmSim(motorModel,
                    GEARING,
                    ArmConstants.MOMENT_OF_INERTIA,
                    ArmConstants.ARM_LENGTH,
                    ArmConstants.MIN_ANGLE_RADS,
                    ArmConstants.MAX_ANGLE_RADS,
                    true,
                    ArmConstants.START_ANGLE_RADS);

            // encodersim allows us to set the encoder values
            encoderSim = new DutyCycleEncoderSim(encoder);
            encoderSim.setDistance(ArmConstants.START_ANGLE_RADS);

            // put the display on the SmartDashboard
            SmartDashboard.putData("ArmSim", wristDisplay);
            SmartDashboard.putData("arm pid", pid);
        }

		// some checks for the arm position
		if (getAngleRad() < ArmConstants.MIN_ANGLE_RADS - 0.005 || getAngleRad() > ArmConstants.MAX_ANGLE_RADS + 0.005)
			throw new RuntimeException("The arm is in a position that should be unreachable. Please double check the arm constants. Found: " + getAngleRad() + ", Expected: " + ArmConstants.stowedSetpoint);
		else if (ArmConstants.ASSERT_AT_SETPOINT && (getAngleRad() < ArmConstants.stowedSetpoint - 0.01 || getAngleRad() > ArmConstants.stowedSetpoint + 0.01))
			throw new RuntimeException("The arm is not at its lowest position. Please move it there and redeploy. If the arm is at its lowest position, check the arm constants. If you know what you're doing, you can override this warning by setting ArmConstants.ASSERT_AT_SETPOINT to false. Found: " + getAngleRad() + ", Expected: " + ArmConstants.stowedSetpoint);
			//System.err.println("WARNING: THE ARM IS NOT AT ITS STOWED SETPOINT.\nIf this was expected, continue. If not, please DOUBLE CHECK THE ARM CONSTANTS or YOU RISK BREAKING THE ARM.");
			// TODO: decide if we want to error or simply display a warning here (or something else)

        // TODO: remove when not needed.
        // Add some test commands
        SmartDashboard.putData("Set Angle to 0.0", new InstantCommand(() -> setAngle(0.0)));
        SmartDashboard.putData("Set Angle to 1.0 Rad", new InstantCommand(() -> setAngle(0.6)));
        SmartDashboard.putData("arm pid", pid);
        if (Constants.DO_LOGGING) {
            LogManager.add("Arm/PositionError", () -> getAngleRad() - pid.getSetpoint(), Duration.ofSeconds(1));
            // pid setpoint and get radians

            ArrayList<Double> slave_errors = new ArrayList<Double>();
            for (TalonFX each_talon: motors) { // could use TalonFX as it originally was. tomato tomahto
                slave_errors.add(each_talon.getPosition().getValue()-motors[0].getPosition().getValue());
            }

            LogManager.add("Arm/SlaveErrors(ticks)", () -> slave_errors);
        }
    }

    /**
     * Get the arm angle using the absolute sensor or the motor encoder.
     * @return
     */
    double getRadians() {
        // using the REV absolute sensor is easy
        return encoder.getDistance();

        // using the motor encoder is more difficult
        // refresh the position (needed because we want it more often than 4 Hz)
        // rotorPositionSignal.refresh();
        // return Units.rotationsToRadians(rotorPositionSignal.getValue() / ArmConstants.GEARING);
    }

    @Override
    public void periodic() {
        // Clamp the PID setpoint in case an out of range value slips in ...
        double setpoint = pid.getSetpoint();
        if (setpoint < ArmConstants.MIN_ANGLE_RADS) {
            pid.setSetpoint(ArmConstants.MIN_ANGLE_RADS);
            setpoint = ArmConstants.MIN_ANGLE_RADS;
        }
        if (setpoint > ArmConstants.MAX_ANGLE_RADS) {
            pid.setSetpoint(ArmConstants.MAX_ANGLE_RADS);
            setpoint = ArmConstants.MAX_ANGLE_RADS;
        }

        // calculate the desired duty cycle
        // if(encoder.getDistance() < ArmConstants.MAX_ANGLE_RADS + .2 && encoder.getDistance() > ArmConstants.MIN_ANGLE_RADS - .2)  {
        dutyCycle = MathUtil.clamp(
                        pid.calculate(getAngleRad()) + feedforward.calculate(pid.getSetpoint(), 0),
                        -1,
                        1);
        // }
        // else 
        // {
        //     for(int i=0;i<motors.length;i++) {
        //         motors[i].setNeutralMode(NeutralModeValue.Coast);
        //     }
            
        // }

        // use the Phoenix 6 version of setting the motor "power"
        motors[0].setControl(m_request.withOutput(dutyCycle));

        // report the arm angle in radians
        SmartDashboard.putNumber("Arm angle", encoder.getDistance());
        SmartDashboard.putNumber("Get Position", getAngleRad());

        // TODO: Clean these up when not needed.
        // report dutycycle
        SmartDashboard.putNumber("arm pow", dutyCycle);

        // these use the motor's internal encoder (not the REV absolute encoder)
        // report the rotor position. This should trigger an update so we get results faster than 4 times per second.)
        // SmartDashboard.putNumber("Rotor Signal", Units.rotationsToRadians(rotorPositionSignal.getValue()));
        // check the latency
        // SmartDashboard.putNumber("Rotor delay", rotorPositionSignal.getTimestamp().getLatency());
        
        // report the absolute position in rotations. Use the abs rotations to set the OFFSET.
        SmartDashboard.putNumber("REV ABS", encoder.getAbsolutePosition());
        // report whether the arm has reached its setpoint
        SmartDashboard.putBoolean("at setpoint?", atSetpoint());
    }

    @Override
    public void simulationPeriodic() {
        // Assuming the volts
        double voltsBattery = 12.8;

        // set the simulator's input voltage
        simulation.setInputVoltage(motors[0].get() * voltsBattery);

        // run the simulator for 20 milliseconds
        simulation.update(0.02);

        // set the arm angle in the display
        moving.setAngle(Units.radiansToDegrees(simulation.getAngleRads()));

        // update the DutyCycleEncoder
        encoderSim.setDistance(simulation.getAngleRads());
    }

    /**
     * Sets the angle of the arm in radians.
     * @param angle (in radians)
     */
    public void setAngle(double angle) {
        // zero the integrator portion of the PID controller
        pid.reset();

        // clamp the angle to avoid problems
        angle = MathUtil.clamp(angle, ArmConstants.MIN_ANGLE_RADS, ArmConstants.MAX_ANGLE_RADS);

        // change the setpoint
        pid.setSetpoint(angle);
    }

    /**
     * Returns the angle of the wrist in radians.
     * @returns arm angle in radians
     */
    public double getAngleRad() {
        return encoder.getDistance();
    }

    /**
     * Returns the angle of the arm in radians.
     * <p>
     * Stop chasing ghosts. This method just re-implements encoder.getDistance().
     * @return arm angle in radians with zero being horizontal.
     * @Deprecated use getAngleRad()
     */
    @Deprecated
    public double getPosition()  {
        return -Units.rotationsToRadians(encoder.getAbsolutePosition() - encoder.getPositionOffset());
    }

    /**
     * Whether the arm has reached its setpoint
     * @return true when the setpoint is reached.
     */
    public boolean atSetpoint() {
        return pid.atSetpoint();
    }
}
