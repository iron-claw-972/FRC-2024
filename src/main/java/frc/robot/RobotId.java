package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.drive.Module;

import java.util.List;

/**
 * Set of known Robot Names.
 * <p>The name of a robot in the RoboRIO's persistent memory.
 * At deploy time, that name is used to set the corresponding RobotId.
 * <p>Note that the RobotId is determined at Deploy time.
 */
public enum RobotId {
    Default,
    SwerveCompetition(Drivetrain.class, Module.class), SwerveTest(Drivetrain.class),
    ClassBot1, ClassBot2, ClassBot3, ClassBot4;

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

    public boolean isClassBot() {
        return this == ClassBot1 || this == ClassBot2 || this == ClassBot3 || this == ClassBot4;
    }

    public boolean isSwerveBot() {
        return this == SwerveCompetition || this == SwerveTest;
    }
    

}
