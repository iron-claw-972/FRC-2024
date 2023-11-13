package frc.robot.subsystems;

import java.lang.reflect.Array;
import java.util.Arrays;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotId;

public class SubsystemFactory {

    @SafeVarargs
    @SuppressWarnings("unchecked")
    public static <T> T get(Class<T> clazz, Pair<Class<?>, Object>... args) {
        RobotId robotId = Robot.getRobotId();
        Class<?>[] parameterTypes = Arrays.stream(args).map(Pair::getFirst).toArray(Class<?>[]::new);
        Object[] argValues = Arrays.stream(args).map(Pair::getSecond).toArray(Object[]::new);
        try {
            if (robotId.getSubsystems().contains(clazz) && RobotBase.isReal()) {
                Class<? extends SubsystemBase> impl = clazz.getAnnotation(SubsystemImpl.class).value();
                return (T) impl.getDeclaredConstructor(parameterTypes).newInstance(argValues);
            }
            return clazz.getDeclaredConstructor(parameterTypes).newInstance(argValues);
        } catch (Exception e) {
            DriverStation.reportError("Could not create subsystem " + clazz.getSimpleName(), e.getStackTrace());
        }
        return null;
    }

}