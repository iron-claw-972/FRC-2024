package frc.robot.subsystems.gpm;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.IntakeConstants;

public class Intake extends SubsystemBase {

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
    private static final DCMotor dcMotor = DCMotor.getNeoVortex(1);

    /** Centering motor is a NEO */
    private final CANSparkMax centeringMotor = new CANSparkMax(IntakeConstants.CENTERING_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
    private static final DCMotor dcMotorCentering = DCMotor.getNEO(1);
    
    /** beam break sensor detects whether a note is present */
    private final DigitalInput sensor  = new DigitalInput(IntakeConstants.SENSOR_ID);

    private double motorRPMSim;
    private double centeringMotorRPMSim;

    private FlywheelSim flywheelSim;
    private FlywheelSim centeringFlywheelSim;

    private Mode mode;

    public Intake() {
        // set the motor parameters
        motor.setIdleMode(IntakeConstants.idleMode);
        centeringMotor.setIdleMode(IntakeConstants.idleMode);

        // set the mode to Idle; this will turn off the motors
        setMode(Mode.DISABLED);

        // digital inputs
        // addChild("Intake motor", motor);
        // addchild("Centering motor", centeringMotor);
        addChild("Intake sensor", sensor);

        // Simulation objects
        if (RobotBase.isSimulation()) {
            // assuming gearing is 1:1 for both
            flywheelSim = new FlywheelSim(dcMotor, 1.0, IntakeConstants.MOI_TOTAL);
            centeringFlywheelSim = new FlywheelSim(dcMotorCentering ,  1.0, IntakeConstants.MOI_CENTERING_TOTAL);
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
