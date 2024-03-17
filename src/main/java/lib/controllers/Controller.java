package lib.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import java.util.function.BooleanSupplier;

public class Controller {
    protected final Joystick controller;

    public Controller(int port) {
        this.controller = new Joystick(port);
    }

    public Trigger get(BooleanSupplier sup) {
        return new Trigger(sup);
    }

    public enum RumbleStatus {
        RUMBLE_ON(1),
        RUMBLE_OFF(0);

        public final double rumbleValue;

        RumbleStatus(final double rumbleValue) {
            this.rumbleValue = rumbleValue;
        }
    }
    public void setRumble(RumbleStatus rumbleStatus) {
        controller.setRumble(RumbleType.kBothRumble, rumbleStatus.rumbleValue);
    }
}
