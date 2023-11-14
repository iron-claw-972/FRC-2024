package frc.robot.subsystems.drive;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.swerve.ModuleConstants;
import frc.robot.subsystems.SubsystemFactory;
import frc.robot.subsystems.SubsystemImpl;

import java.util.Arrays;

/**
 * This class should include methods that will be overridden by {@link DrivetrainImpl} to be used on an actual robot
 * with a swerve style drivetrain. This class will be used when the current robot does NOT have a swerve style
 * drivetrain, as defined in {@link frc.robot.RobotId}.
 */
@SubsystemImpl(DrivetrainImpl.class)
public class Drivetrain extends SubsystemBase {

    protected final Module[] modules;
//    protected final ModuleImpl[] modules;

    public Drivetrain() {
        modules = new Module[4];
        ModuleConstants[] constants = Arrays.copyOfRange(ModuleConstants.values(), 0, 4);
        Arrays.stream(constants).forEach(moduleConstants -> {
            modules[moduleConstants.ordinal()] = SubsystemFactory.get(Module.class, new Pair<>(ModuleConstants.class, moduleConstants));
        });
//        modules = new ModuleImpl[]{
//            new ModuleImpl(ModuleConstants.FRONT_LEFT),
//            new ModuleImpl(ModuleConstants.FRONT_RIGHT),
//            new ModuleImpl(ModuleConstants.BACK_LEFT),
//            new ModuleImpl(ModuleConstants.BACK_RIGHT)
//        };
    }
    

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        speed of the robot in the x direction (forward) in m/s
     * @param ySpeed        speed of the robot in the y direction (sideways) in m/s
     * @param rot           angular rate of the robot in rad/s
     * @param fieldRelative whether the provided x and y speeds are relative to the field
     * @param isOpenLoop    whether to use velocity control for the drive motors
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean isOpenLoop) {
        // TODO: Implement what this method should do as a placeholder
    }

    public Module[] getModules() {
        return modules;
    }

}
