package frc.robot.subsystems.gpm;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkFlex;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DIOSim;
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

    private int countSim = 0;
    private DIOSim IntakeSensorDioSim;
    private boolean simDIOValue = true;

    // private XboxController m_gc;

    private Mode mode;

    public Intake() {
        motor = new CANSparkFlex(IntakeConstants.MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
        centeringMotor = new CANSparkMax(IntakeConstants.CENTERING_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
        sensor = new DigitalInput(IntakeConstants.SENSOR_ID);
        mode = Mode.DISABLED;

        // digital inputs
        addChild("Intake sensor", sensor);

        // //Simulation objects
        if (RobotBase.isSimulation()) {
            IntakeSensorDioSim = new DIOSim(sensor);
        }

        publish();
    }

    // publish sensor to Smart Dashboard
    private void publish() {
        SmartDashboard.putBoolean("Intake Sensor", sensor.get());
    }

    public void setMode(Mode mode) {
        this.mode = mode;
    }

    public boolean hasNote() {
        return !sensor.get();
    }

    @Override
    public void periodic() {
        motor.set(mode.getPower());
        centeringMotor.set(mode.getCenteringPower());

        publish();
    }

    public double getCurrent() {
        return Math.abs(motor.getOutputCurrent());
    }

    public double getCenteringCurrent() {
        return Math.abs(centeringMotor.getOutputCurrent());
    }

    @Override
    public void simulationPeriodic() {

        // change values every 1/2 secound
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
