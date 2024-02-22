package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import lib.controllers.GameController;
import lib.controllers.GameController.RumbleStatus;

/**
 * Sets a controller to rumble for a specified amount of time
 */
public class RumbleCommand extends SequentialCommandGroup {
    /**
     * Makes a controller rumble
     * @param controller The controller to make rumble
     * @param time The time, in seconds, to rumble for
     */
    public RumbleCommand(GameController controller, double time){
        addCommands(
            new InstantCommand(()->controller.setRumble(RumbleStatus.RUMBLE_ON)),
            new WaitCommand(time).finallyDo(
                (interruped)->controller.setRumble(RumbleStatus.RUMBLE_OFF))
        );
    }
}
