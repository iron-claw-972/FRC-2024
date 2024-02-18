package frc.robot.subsystems.gpm;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkFlex;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.IntakeConstants;

public class Intake extends SubsystemBase {

    public enum Mode {
        INTAKE(IntakeConstants.INTAKE_POWER, IntakeConstants.CENTERING_POWER),
        DISABLED(0, 0);

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

    private final CANSparkFlex motor;
    private final CANSparkMax centeringMotor;
    private final DigitalInput sensor;

    private final double MASS_SHAFT = 0.4; // in kilograms
    private final double LENGTH_SHAFT = Units.inchesToMeters(25.5);
    private final double MOI_SHAFT = (1.0 / 12.0) * MASS_SHAFT * LENGTH_SHAFT * LENGTH_SHAFT;
    private final double MOI_TOTAL = MOI_SHAFT * 4;

    private final double MASS_CENTERING_WHEELS = 0.1; // in kilograms
    private final double RADIUS_CENTERING_WHEELS = Units.inchesToMeters(2);
    private final double MOI_CENTERING_WHEEL = 0.5 * MASS_CENTERING_WHEELS * RADIUS_CENTERING_WHEELS
            * RADIUS_CENTERING_WHEELS;
    private final double MOI_CENTERING_TOTAL = MOI_CENTERING_WHEEL * 4;

    private final double motorVoltage = 12.0;

    private double motorRPMSim;
    private double motorPower;
    private double centeringMotorPower;
    private double centeringMotorRPMSim;

    private int countSim = 0;
    private DIOSim IntakeSensorDioSim;
    private boolean simDIOValue = true;
    private FlywheelSim flywheelSim;
    private FlywheelSim centeringFlywheelSim;

    private Mode mode;

    public Intake() {
        motor = new CANSparkFlex(IntakeConstants.MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
        centeringMotor = new CANSparkMax(IntakeConstants.CENTERING_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
        sensor = new DigitalInput(IntakeConstants.SENSOR_ID);
        mode = Mode.DISABLED;

        // digital inputs
        addChild("Intake sensor", sensor);

        // Simulation objects
        if (RobotBase.isSimulation()) {
            IntakeSensorDioSim = new DIOSim(sensor);
            // assuming gearing is 1:1 for both
            flywheelSim = new FlywheelSim(DCMotor.getNeoVortex(1), 1.0, MOI_TOTAL);
            centeringFlywheelSim = new FlywheelSim(DCMotor.getNeo550(1), 1.0, MOI_CENTERING_TOTAL); // change the motor
                                                                                                    // from neo550 to
                                                                                                    // whatever it
                                                                                                    // actually is
        }

        publish();
    }

    // publish sensor to Smart Dashboard
    private void publish() {
        SmartDashboard.putBoolean("Intake Sensor", sensor.get());
        if (RobotBase.isSimulation()) {
            SmartDashboard.putNumber("Intake motor RPM", motorRPMSim);
            SmartDashboard.putNumber("Intake centering motor RPM", centeringMotorRPMSim);
        }
    }

    public void setMode(Mode mode) {
        this.mode = mode;
    }

    public boolean hasNote() {
        return !sensor.get();
    }

    @Override
    public void periodic() {
        motorPower = mode.getPower();
        centeringMotorPower = mode.getCenteringPower();

        if (RobotBase.isReal()) {
            motor.set(motorPower);
            centeringMotor.set(centeringMotorPower);
        }

        publish();
    }

    public double getCurrent() {
        if (RobotBase.isReal()) {
            return Math.abs(motor.getOutputCurrent());
        } else {
            return motorPower / motorVoltage;
        }
    }

    public double getCenteringCurrent() {
        if (RobotBase.isReal()) {
            return Math.abs(centeringMotor.getOutputCurrent());
        } else {
            return centeringMotorPower / motorVoltage;
        }
    }

    @Override
    public void simulationPeriodic() {
        flywheelSim.setInputVoltage(motorPower * motorVoltage);
        centeringFlywheelSim.setInputVoltage(centeringMotorPower * motorVoltage);

        flywheelSim.update(0.020);
        centeringFlywheelSim.update(0.020);

        motorRPMSim = flywheelSim.getAngularVelocityRPM();
        centeringMotorRPMSim = centeringFlywheelSim.getAngularVelocityRPM();

        // change values every 1/2 second
        if (countSim++ > 25) {
            countSim = 0;

            IntakeSensorDioSim.setValue(simDIOValue);

            simDIOValue = !simDIOValue;
        }

    }

    public void close() {
        sensor.close();
    }
}
