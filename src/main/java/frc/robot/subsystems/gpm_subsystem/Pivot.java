package frc.robot.subsystems.gpm_subsystem;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.globalConst;
import frc.robot.constants.PivotConstants;
import frc.robot.util.MotorFactory;

public class Pivot extends SubsystemBase {

    public enum Mode {
        STOW(PivotConstants.STOW_POS), INTAKE(PivotConstants.INTAKE_POS);

        private double setpoint;

        Mode(double setpoint) {
            this.setpoint = setpoint;
        }

        public double getSetpoint() {
            return setpoint;
        }
    }

    private final WPI_TalonFX motor;
    private final PIDController pid;

    protected final DutyCycleEncoder encoder;

    private double lastPos = 0;

    public Pivot() {
        // configure the motor.
        motor = MotorFactory.createTalonFXSupplyLimit(
                PivotConstants.MOTOR_ID,
                globalConst.RIO_CAN,
                PivotConstants.CONTINUOUS_CURRENT_LIMIT,
                PivotConstants.PEAK_CURRENT_LIMIT,
                PivotConstants.PEAK_CURRENT_DURATION);
        motor.setNeutralMode(PivotConstants.NEUTRAL_MODE);
        motor.setInverted(PivotConstants.MOTOR_INVERT);
        motor.enableVoltageCompensation(true);

        // config deadband to be less, may be powering at small values to keep it up
        motor.configNeutralDeadband(0.005);

        // configure the encoder
        encoder = new DutyCycleEncoder(PivotConstants.ABS_ENCODER_PORT);
        // Cleaner encoder implementation
        // offset to zero (arm horizontal)
        encoder.setPositionOffset(PivotConstants.ENCODER_OFFSET);
        // scale to radians and invert direction
        encoder.setDistancePerRotation(2.0 * Math.PI);

        // make the PID controller
        pid = new PIDController(PivotConstants.P, PivotConstants.I, PivotConstants.D);
        // set the PID controller's tolerance
        pid.setTolerance(PivotConstants.TOLERANCE);

        setMode(Mode.STOW);
    }

    public void setMode(Mode mode) {
        // set the PID integration error to zero.
        pid.reset();
        // set the PID desired position
        pid.setSetpoint(mode.getSetpoint());
    }

    @Override
    public void periodic() {
        // obtain the wrist position
        double position = getAbsEncoderPos();

        // calculate the PID power level
        // for safety, clamp the setpoint to prevent tuning with SmartDashboard/Shuffleboard from commanding out of range
        // This method continually changes the setpoint.
        double pidPower = pid.calculate(position, MathUtil.clamp(pid.getSetpoint(), PivotConstants.MIN_POS,
                PivotConstants.MAX_POS));

        // calculate the value of kGravityCompensation
        double feedforwardPower = PivotConstants.GRAVITY_COMPENSATION * Math.cos(position);

        // set the motor power
        setMotorPower(pidPower + feedforwardPower);
    }

    /**
     * Whether the wrist has reached its commanded position.
     * @returns true when position has been reached
     */
    public boolean reachedSetpoint() {
        return pid.atSetpoint();
    }

    /**
     * Sets the motor power, clamping it and ensuring it will not activate below/above the min/max positions
     */
    private void setMotorPower(double power) {

        power = MathUtil.clamp(power, -PivotConstants.MOTOR_POWER_CLAMP, PivotConstants.MOTOR_POWER_CLAMP);

        double pos = getAbsEncoderPos();

        // double safety check incase encoder outputs weird values again
        if (pos <= PivotConstants.MIN_POS && power < 0) {
            power = 0;
        }
        if (pos >= PivotConstants.MAX_POS && power > 0) {
            power = 0;
        }

        motor.set(power);
    }

    /**
     * @return the absolute encoder position in rotations, zero being facing forward
     */
    public double getAbsEncoderPos() {
        double pos = encoder.getDistance();
        if (pos > PivotConstants.MAX_POS || pos < PivotConstants.MIN_POS) {
            pos = lastPos;
        }
        lastPos = pos;
        return pos;
    }

}
