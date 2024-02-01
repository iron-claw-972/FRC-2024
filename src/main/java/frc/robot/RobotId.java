package frc.robot;

/**
 * Set of known Robot Names.
 * <p>The name of a robot in the RoboRIO's persistent memory.
 * At deploy time, that name is used to set the corresponding RobotId.
 * <p>Note that the RobotId is determined at Deploy time.
 */
public enum RobotId {
    Default,
    SwerveCompetition, SwerveTest,
    ClassBot1, ClassBot2, ClassBot3, ClassBot4;

    public boolean isClassBot() {
        return this == ClassBot1 || this == ClassBot2 || this == ClassBot3 || this == ClassBot4;
    }

    public boolean isSwerveBot() {
        return this == SwerveCompetition || this == SwerveTest;
    }
}
