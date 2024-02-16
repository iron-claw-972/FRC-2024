package frc.robot.subsystems.gpm;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.IntakeConstants;


public class Intake extends SubsystemBase {

    public enum Mode {
        INTAKE(IntakeConstants.INTAKE_POWER), DISABLED(0)
        ;

        private double power;

        Mode(double power) {
            this.power = power;
        }

        public double getPower() {
            return power;
        }
    }

    private final CANSparkFlex motor;
    private final DigitalInput sensor;

    private int countSim = 0;
    private DIOSim IntakeSensorDioSim;
    private  boolean foo = true;

    private Mode mode;


    public Intake() {
        motor = new CANSparkFlex(IntakeConstants.MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
        sensor = new DigitalInput(IntakeConstants.SENSOR_ID);
        mode = Mode.DISABLED;

        //digital inputs
        addChild("Intake sensor", sensor);

//         //Simulation objects
        if (RobotBase.isSimulation()) {
            IntakeSensorDioSim = new DIOSim(sensor);
        }

        publish();
    }

     //publish sensor to Smart Dashboard
    private void publish() {
        SmartDashboard.putBoolean("Intake Sensor", sensor.get());
    }

    public void setMode(Mode mode) {
        this.mode = mode;
    }

    public boolean hasNote() {
        return sensor.get();
    }

    @Override
    public void periodic() {
        motor.set(mode.getPower());

         publish();

        if (!sensor.get()) {
            System.out.println("Object detected");

        } else if (sensor.get()) {
            System.out.println("No object detected");

        } else {
            System.out.println("=P");
        }
    }

    public double getCurrent() {
        return Math.abs(motor.getOutputCurrent());
    }

    @Override
    public void simulationPeriodic() {

        //change values every 1/2 secound
         if (countSim++ > 25) {
            countSim = 0;

            IntakeSensorDioSim.setValue(foo);

            foo = ! foo;
         }

    }

     public void close() {
            sensor.close();
    }
}
