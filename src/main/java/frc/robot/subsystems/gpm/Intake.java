package frc.robot.subsystems.gpm;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        DISABLED(0,0),
        INTAKE(.3,.3),
        REVERSE(-.3,-.3);

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

    /** Intake motor is a Vortex*/
    private final CANSparkFlex motor = new CANSparkFlex(IntakeConstants.MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);

    // change the motor from neo550 to whatever it actually is

    /** Centering motor is a NEO */
    private final CANSparkMax centeringMotor = new CANSparkMax(IntakeConstants.CENTERING_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
    
    /** beam break sensor detects whether a note is present */
    private final DigitalInput sensor  = new DigitalInput(IntakeConstants.SENSOR_ID);

    private double motorRPMSim;
    private double centeringMotorRPMSim;

    private FlywheelSim flywheelSim;
    private FlywheelSim centeringFlywheelSim;

    private Mode mode;

    public Intake() {
        // set the motor parameters
        // motor.setIdleMode(idleMode);
        centeringMotor.setIdleMode(idleMode);

        setMode(Mode.DISABLED);

        // digital inputs
        // addChild("Intake motor", motor);
        // addchild("Centering motor", centeringMotor);
        addChild("Intake sensor", sensor);

        // Simulation objects
        if (RobotBase.isSimulation()) {
            // assuming gearing is 1:1 for both
            flywheelSim = new FlywheelSim(IntakeConstants.dcMotor, 1.0, IntakeConstants.MOI_TOTAL);
            centeringFlywheelSim = new FlywheelSim(IntakeConstants.dcMotorCentering ,  1.0, IntakeConstants.MOI_CENTERING_TOTAL);
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

        // set the motor powers to be the value appropriate for this mode
        motor.set(mode.power);
        centeringMotor.set(mode.centeringPower);
    }

    public boolean hasNote() {
        return !sensor.get();
    }

    @Override
    public void periodic() {
        publish();
    }

    @Override
    public void simulationPeriodic() {
        flywheelSim.setInputVoltage(mode.power * Constants.ROBOT_VOLTAGE);
        centeringFlywheelSim.setInputVoltage(mode.centeringPower * Constants.ROBOT_VOLTAGE);

        flywheelSim.update(0.020);
        centeringFlywheelSim.update(0.020);

        motorRPMSim = flywheelSim.getAngularVelocityRPM();
        centeringMotorRPMSim = centeringFlywheelSim.getAngularVelocityRPM();
    }

    public void close() {
        sensor.close();
    }
}
