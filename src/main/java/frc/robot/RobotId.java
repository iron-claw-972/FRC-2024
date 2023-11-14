package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Drivetrain;

import java.util.List;

/**
 * Set of known Robot Names.
 * <p>The name of a robot in the RoboRIO's persistent memory.
 * At deploy time, that name is used to set the corresponding RobotId.
 * <p>Note that the RobotId is determined at Deploy time.
 */
public enum RobotId {
    Default,
    Competition(Drivetrain.class), Test(),
    ;

    /**
     * List of subsystems to create when the robot is instantiated.
     */
    private final List<Class<? extends SubsystemBase>> subsystems;

    @SafeVarargs
    RobotId(Class<? extends SubsystemBase>... subsystems) {
        this.subsystems = List.of(subsystems);
    }

    public List<Class<? extends SubsystemBase>> getSubsystems() {
        return subsystems;
    }

}
