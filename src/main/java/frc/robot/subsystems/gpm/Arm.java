package frc.robot.subsystems.gpm;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
    private static final DCMotor motorModel = DCMotor.getKrakenX60(ArmConstants.MOTOR_IDS.length);

    /**
     * REV absolute encoder.
     * <p>
     *  It is mounted on the right hand side. CCW rotation is arm raises.
     */
    private final DutyCycleEncoder encoder = new DutyCycleEncoder(ArmConstants.ENCODER_ID);
    /** this instance sets the REV absolute encoder value during simulations */
    private DutyCycleEncoderSim encoderSim;
    /** REV encoder offset in radians */
    private static final double OFFSET = 0;
    /** REV encoder scale factor */
    private static final double DISTANCE_PER_ROTATION = 2 * Math.PI;


    // Motor PID control
    private static final double TOLERANCE = Units.degreesToRadians(1.0);
    private static final double P = 5;
    private static final double I = 0;
    private static final double D = 0;
    private final PIDController pid = new PIDController(P, I, D);

    // Motor feedforward control
    public static final double S = 0;
    public static final double V = 0;
    // TODO: simpleMotorFeedforward does not do gravity!
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(S, V);

    /** arm simulator */
    private SingleJointedArmSim simulation;

    // Mechanism display
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

        // set the PID initial position
        // TODO: figure this out some more.
        pid.setSetpoint(ArmConstants.START_ANGLE_RADS);

        // make the encoder report arm angle in radians
        encoder.setDistancePerRotation(DISTANCE_PER_ROTATION);
        encoder.setPositionOffset(OFFSET);
   
        // consider each of the motors
        for (int i = 0; i < motors.length; i++) {
            // create the motor
            motors[i] = new TalonFX(ArmConstants.MOTOR_IDS[i]);

            // common configuration for each motor
            // TODO: should these be sent only to the master after the slaves are configured?
            motors[i].setNeutralMode(ArmConstants.neutralMode);
            motors[i].setInverted(ArmConstants.inverted);
            motors[i].getConfigurator().apply(ArmConstants.currentConfig);

            // i=0 is the master; the others are slaves
            if (i > 0) {
                // set the slave mode
                // the master is MOTOR_IDS[0]
                // invert master for ids 2 and 3
                motors[i].setControl(new Follower(ArmConstants.MOTOR_IDS[0], (i >= motors.length / 2)));
            }
        }

        // possibly set up simulations
        if (RobotBase.isSimulation()) {
            simulation = new SingleJointedArmSim(motorModel,
                    ArmConstants.GEARING,
                    ArmConstants.MOMENT_OF_INERTIA,
                    ArmConstants.ARM_LENGTH,
                    ArmConstants.MIN_ANGLE_RADS,
                    ArmConstants.MAX_ANGLE_RADS,
                    true,
                    ArmConstants.START_ANGLE_RADS);

            // encodersim allows us to set the encoder values
            encoderSim = new DutyCycleEncoderSim(encoder);

            // put the display on the SmartDashboard
            SmartDashboard.putData("ArmSim", wristDisplay);
            SmartDashboard.putData(pid);
        }

        // Add some test commands
        SmartDashboard.putData("Set Angle to 0.0", new InstantCommand(() -> setAngle(0.0)));
        SmartDashboard.putData("Set Angle to 1.0 Rad", new InstantCommand(() -> setAngle(1.0)));
    }

    @Override
    public void periodic() {
        // use the scaled distance (which is radians)
        motors[0].set(
                MathUtil.clamp(
                        pid.calculate(encoder.getDistance()) + feedforward.calculate(pid.getSetpoint()),
                        -1,
                        1));

        // report the arm angle
        SmartDashboard.putNumber("Arm angle", Units.radiansToDegrees(encoder.getDistance()));
    }

    @Override
    public void simulationPeriodic() {
        // Assuming the volts
        double voltsBattery = 12.8;

        // set the simulator inputs
        simulation.setInputVoltage(motors[0].get() * voltsBattery);

        // run the simulator for 20 milliseconds
        simulation.update(0.02);

        // set the arm angle in the display
        moving.setAngle(Units.radiansToDegrees(simulation.getAngleRads()));

        // update the DutyCycleEncoder
        encoderSim.setDistance(simulation.getAngleRads());
    }

    /**
     * Sets the angle of the wrist in radians.
     * @param angle (in radians)
     */
    public void setAngle(double angle) {
        // zero the integrator portion of the PID controller
        pid.reset();
        // change the setpoint

        pid.setSetpoint(angle);
    }

    /**
     * Returns the angle of the wrist in radians.
     * @returns arm angle in radians
     */
    public double getAngleRad() {
        return encoder.getDistance() / ArmConstants.GEARING;
    }

    /**
     * Whether the arm has reached its setpoint
     * @return true when the setpoint is reached.
     */
    public boolean atSetpoint() {
        return pid.atSetpoint();
    }
}