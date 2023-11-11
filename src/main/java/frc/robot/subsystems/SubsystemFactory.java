package frc.robot.subsystems;

import java.util.Arrays;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotId;

public class SubsystemFactory {

    /**
     * Creates a subsystem based on the subsystems current robot's subsystems as defined in {@link RobotId}. If the
     * current {@link RobotId} does not include the requested subsystem, then a simulated filler subsystem will be
     * created. If it does, then an actual implementation of the subsystem with motors will be created.
     * <br>
     * <br>
     * Parameters of this method should be the class of the subsystem to create, not the class of the implementation.
     * For example, {@link Drivetrain} would be used as the parameter, not {@link DrivetrainImpl}.
     *
     * @param clazz the class of the subsystem to create
     * @return the created subsystem
     */
    public static SubsystemBase get(Class<?> clazz) {
        RobotId robotId = Robot.getRobotId();
        Class<?>[] parameterTypes = Arrays.stream(args).map(Object::getClass).toArray(Class<?>[]::new);
        try {
            if (robotId.getSubsystems().contains(clazz) || RobotBase.isSimulation()) {
                Class<? extends SubsystemBase> impl = clazz.getAnnotation(SubsystemImpl.class).value();
                return (T) impl.getDeclaredConstructor(parameterTypes).newInstance(args);
            }  
            return clazz.getDeclaredConstructor(parameterTypes).newInstance(args);       
        } catch (Exception e) {
            DriverStation.reportError("Could not create subsystem " + clazz.getSimpleName(), e.getStackTrace());
        }
        return null;
    }

}
