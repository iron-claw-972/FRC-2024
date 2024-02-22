package frc.robot.subsystems.gpm;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.IntakeConstants;

public class Intake extends SubsystemBase {
    // Current limits -- not used
    // private static final int CONTINUOUS_CURRENT_LIMIT = 25;
    // private static final int PEAK_CURRENT_LIMIT = 55;
    // private static final double PEAK_CURRENT_DURATION = 0.1;
    // private static final boolean ENABLE_CURRENT_LIMIT = true;

    // private static final double INTAKE_STALL_TIME = 0.2;
    // private static final double INTAKE_CURRENT_STOP = 10;

    private static final IdleMode idleMode = IdleMode.kBrake;

    public enum Mode {
        DISABLED(0, 0),
        INTAKE(.3, .3),
        REVERSE(-.3, -.3);

        private double power;
        private double centeringPower;

        Mode(double power, double centeringPower) {
            this.power = power;
            this.centeringPower = centeringPower;
        }

        public double getPower() {
            return power;
        }

        public double getCenteringPower() {
            return centeringPower;
        }
    }

    /** Intake motor is a Vortex */
    private final CANSparkFlex motor = new CANSparkFlex(IntakeConstants.MOTOR_ID,
            CANSparkLowLevel.MotorType.kBrushless);
    private static DCMotor dcMotor = DCMotor.getNeoVortex(1);

    // change the motor from neo550 to whatever it actually is

    /** Centering motor is a NEO */
    private final CANSparkMax centeringMotor = new CANSparkMax(IntakeConstants.CENTERING_MOTOR_ID,
            CANSparkLowLevel.MotorType.kBrushless);
    private static DCMotor dcMotorCentering = DCMotor.getNEO(1);

    /** beam break sensor detects whether a note is present */
    private final DigitalInput sensor = new DigitalInput(IntakeConstants.SENSOR_ID);
    /** Allows us to simulate the beam break sensor */
    private DIOSim sensorSim;

    // Timer for simulator
    private Timer simTimer = new Timer();

    private double motorRPMSim;
    private double centeringMotorRPMSim;

    private FlywheelSim flywheelSim;
    private FlywheelSim centeringFlywheelSim;

    private Debouncer noteDebouncer = new Debouncer(2, DebounceType.kRising);
    private Debouncer reverseDebouncer = new Debouncer(0.5, DebounceType.kFalling);
    private Command jamCommand = new PrintCommand("A NOTE IS JAMMED IN THE INTAKE");
    private boolean jammed = false;

    // MOI stuff
    // Intake rollers are 1.5 inch polycarb. We are ignoring the weight of the black
    // tape.
    public static final double MASS_SHAFT = 0.4; // in kilograms
    public static final double RADIUS_SHAFT = Units.inchesToMeters(0.75);
    public static final double MOI_SHAFT = MASS_SHAFT * RADIUS_SHAFT * RADIUS_SHAFT;
    public static final double MOI_TOTAL = MOI_SHAFT * 4;

    // The centering rollers are compliant wheels. Assume 1/2 the mass is at the
    // rim.
    public static final double MASS_CENTERING_WHEELS = 0.1018; // in kilograms
    public static final double RADIUS_CENTERING_WHEELS = Units.inchesToMeters(2);
    public static final double MOI_CENTERING_WHEEL = (0.5 * MASS_CENTERING_WHEELS) * RADIUS_CENTERING_WHEELS
            * RADIUS_CENTERING_WHEELS;
    public static final double MOI_CENTERING_TOTAL = MOI_CENTERING_WHEEL * 4;

    private Mode mode;

    public Intake() {
        // set the motor parameters
        motor.setIdleMode(idleMode);
        centeringMotor.setIdleMode(idleMode);

        setMode(Mode.DISABLED);

        // digital inputs
        // addChild("Intake motor", motor);
        // addchild("Centering motor", centeringMotor);
        addChild("Intake sensor", sensor);

        // Simulation objects
        if (RobotBase.isSimulation()) {
            // assuming gearing is 1:1 for both
            flywheelSim = new FlywheelSim(dcMotor, 1.0, MOI_TOTAL);
            centeringFlywheelSim = new FlywheelSim(dcMotorCentering, 2.0, MOI_CENTERING_TOTAL);
            // code to fake the beam break sensor
            sensorSim = new DIOSim(sensor);
            // the beam is present.
            sensorSim.setValue(true);
        }

        publish();
    }

    // publish sensor to Smart Dashboard
    private void publish() {
        SmartDashboard.putBoolean("Intake Sensor", sensor.get());
        SmartDashboard.putNumber("Intake motor RPM", motorRPMSim);
        SmartDashboard.putNumber("Intake centering motor RPM", centeringMotorRPMSim);
    }

    public void setMode(Mode mode) {
        this.mode = mode;
    }

    public boolean hasNote() {
        return !sensor.get();
    }

    @Override
    public void periodic() {
        publish();

        motor.set(mode.power);
        centeringMotor.set(mode.centeringPower);

        // If it has a note for too long
        if(noteDebouncer.calculate(hasNote())){
            // Reverse intake in auto
            if(DriverStation.isAutonomousEnabled()){
                setMode(Mode.REVERSE);
            }else if(!jammed){
                // Run jamCommand once when first jammed
                jamCommand.schedule();
                jammed = true;
            }
        // If it doesn't have a note for a different amount of time
        }else if(!reverseDebouncer.calculate(hasNote())){
            // Reset jamming for next note
            jammed = false;
            // Disable if it reversed in auto
            if(mode == Mode.REVERSE && DriverStation.isAutonomousEnabled()){
                setMode(Mode.DISABLED);
            }
        }
    }

    public void setJamCommand(Command newJamCommand){
        jamCommand = newJamCommand;
    }

    @Override
    public void simulationPeriodic() {
        flywheelSim.setInputVoltage(mode.power * Constants.ROBOT_VOLTAGE);
        centeringFlywheelSim.setInputVoltage(mode.centeringPower * Constants.ROBOT_VOLTAGE);

        flywheelSim.update(0.020);
        centeringFlywheelSim.update(0.020);

        motorRPMSim = flywheelSim.getAngularVelocityRPM();
        centeringMotorRPMSim = centeringFlywheelSim.getAngularVelocityRPM();

        if(mode == Mode.INTAKE){
            simTimer.start();
            sensorSim.setValue(!simTimer.hasElapsed(1.5) || simTimer.hasElapsed(2));
        }else{
            simTimer.reset();
            simTimer.stop();
        }
    }

    public void close() {
        sensor.close();
    }
}
