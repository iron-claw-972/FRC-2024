package lib.ctre_shims;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

import java.util.Arrays;

/**
 * Allows multiple {@link MotorController} objects to be linked together.
 *
 * <p>Reimplementation with supports CTRE CAN follower functionality.
 */
public class PhoenixMotorControllerGroup implements MotorController, Sendable, AutoCloseable {
    private final MotorController[] motorControllers;
    private final MotorController leadMotorController;
    private static int instances;

    /**
     * Create a new PhoenixMotorControllerGroup with the provided MotorControllers.
     *
     * @param leadMotorController The lead MotorController to add
     * @param motorControllers    The MotorControllers to add
     */
    public PhoenixMotorControllerGroup(
            MotorController leadMotorController, MotorController... motorControllers) {
        boolean validArgumentTypes = leadMotorController instanceof BaseMotorController;
        for (MotorController motorController : motorControllers) {
            validArgumentTypes = validArgumentTypes && (motorController instanceof BaseMotorController);
        }

        if (!validArgumentTypes) {
            throw new IllegalArgumentException(
                    "One or more MotorControllers do not inherit from BaseMotorController, i.e. are not CTRE classes");
        }
        this.leadMotorController = leadMotorController;
        this.motorControllers = Arrays.copyOf(motorControllers, motorControllers.length);
        init();
    }

    private void init() {
        SendableRegistry.addChild(this, leadMotorController);
        for (MotorController controller : motorControllers) {
            SendableRegistry.addChild(this, controller);
            ((BaseMotorController) controller).follow((BaseMotorController) leadMotorController);
            ((BaseMotorController) controller).setInverted(InvertType.FollowMaster);
        }
        instances++;
        SendableRegistry.addLW(this, "PhoenixMotorControllerGroup", instances);
    }

    @Override
    public void close() {
        SendableRegistry.remove(this);
    }

    @Override
    public void set(double speed) {
        leadMotorController.set(speed);
    }

    @Override
    public double get() {
        return leadMotorController.get();
    }

    @Override
    public void setInverted(boolean isInverted) {
        leadMotorController.setInverted(isInverted);
    }

    @Override
    public boolean getInverted() {
        return leadMotorController.getInverted();
    }

    @Override
    public void disable() {
        leadMotorController.disable();
        for (MotorController motorController : motorControllers) {
            motorController.disable();
        }
    }

    @Override
    public void stopMotor() {
        leadMotorController.stopMotor();
        for (MotorController motorController : motorControllers) {
            motorController.stopMotor();
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Phoenix Motor Controller");
        builder.setActuator(true);
        builder.setSafeState(this::stopMotor);
        builder.addDoubleProperty("Value", this::get, this::set);
    }

    @Override
    public void setVoltage(double outputVolts) {
        leadMotorController.setVoltage(outputVolts);
        for (MotorController motorController : motorControllers) {
            motorController.setVoltage(outputVolts);
        }
    }
}
