package frc.robot.subsystems.drive;

import com.ctre.phoenix.sensors.Pigeon2.AxisDirection;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.GlobalConst;
import frc.robot.constants.swerve.DriveConst;
import frc.robot.constants.swerve.ModuleConst;

import java.util.Arrays;

/**
 * Represents a swerve drive style drivetrain.
 * <p>
 * Module IDs are:
 * 1: Front left
 * 2: Front right
 * 3: Back left
 * 4: Back right
 */
public class Drivetrain extends SubsystemBase {

    protected final Module[] modules;

    // Odometry
    //private final SwerveDrivePoseEstimator poseEstimator;

    //private final WPI_Pigeon2 pigeon;

    // PID Controllers for chassis movement
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotationController;
    private final PIDController velocityController;
    // Displays the field with the robots estimated pose on it
    private final Field2d fieldDisplay;

    //testing Vortex motors
    CANSparkFlex leftVortex = new CANSparkFlex(3, MotorType.kBrushless);
    CANSparkFlex rightVortex = new CANSparkFlex(6, MotorType.kBrushless);

    private final ShuffleboardTab swerveTab;
    /**
     * Creates a new Swerve Style Drivetrain.
     */
    public Drivetrain(ShuffleboardTab swerveTab) {
        this.swerveTab = swerveTab;
        modules = new Module[4];
        
        ModuleConst[] constants = Arrays.copyOfRange(ModuleConst.values(), 0, 4);
        
        Arrays.stream(constants).forEach(moduleConstants -> {
           // modules[moduleConstants.ordinal()] = new Module(moduleConstants, swerveTab);
        });

        // // The Pigeon is a gyroscope and implements WPILib's Gyro interface
        // pigeon = new WPI_Pigeon2(DriveConst.kPigeon, DriveConst.kPigeonCAN);
        // pigeon.configFactoryDefault();
        // // Our pigeon is mounted with y forward, and z upward
        // pigeon.configMountPose(AxisDirection.PositiveY, AxisDirection.PositiveZ);

        /*
         * By pausing init for a second before setting module offsets, we avoid a bug
         * with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();
        
        // initial Odometry Location
        // pigeon.setYaw(DriveConst.kStartingHeading.getDegrees());
        // poseEstimator = new SwerveDrivePoseEstimator(
        //         DriveConst.KINEMATICS,
        //         Rotation2d.fromDegrees(pigeon.getYaw()),
        //         getModulePositions(),
        //         new Pose2d() 
        // );
//        poseEstimator.setVisionMeasurementStdDevs(VisionConstants.kBaseVisionPoseStdDevs);
        
        // initialize PID controllers
        xController = new PIDController(DriveConst.kTranslationalP, 0, DriveConst.kTranslationalD);
        yController = new PIDController(DriveConst.kTranslationalP, 0, DriveConst.kTranslationalD);
        rotationController = new PIDController(DriveConst.kHeadingP, 0, DriveConst.kHeadingD);
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        rotationController.setTolerance(Units.degreesToRadians(0.25), Units.degreesToRadians(0.25));

        velocityController = new PIDController(0.5,0,0.01);
        velocityController.setTolerance(0, 0.01);

        fieldDisplay = new Field2d();
        fieldDisplay.setRobotPose(getPose());
    }

    @Override
    public void periodic() {
       // updateOdometry();
        fieldDisplay.setRobotPose(getPose());
    }

    // DRIVE

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
        setChassisSpeeds((
                                 fieldRelative
                                         ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getYaw())
                                         : new ChassisSpeeds(xSpeed, ySpeed, rot)
                         ),
                         isOpenLoop
                        );
    }

    /**
     * Drives the robot using the provided x speed, y speed, and positional heading.
     *
     * @param xSpeed        speed of the robot in the x direction (forward)
     * @param ySpeed        speed of the robot in the y direction (sideways)
     * @param heading       target heading of the robot in radians
     * @param fieldRelative whether the provided x and y speeds are relative to the field
     */
    public void driveHeading(double xSpeed, double ySpeed, double heading, boolean fieldRelative) {
        double rot = rotationController.calculate(getYaw().getRadians(), heading);
        setChassisSpeeds((
                        fieldRelative
                                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getYaw())
                                : new ChassisSpeeds(xSpeed, ySpeed, rot)
                ),
                false
        );
    }

    /**
     * Runs the PID controllers with the provided x, y, and rot values. Then, calls {@link #drive(double, double, double, boolean, boolean)} using the PID outputs.
     * This is based on the odometry of the chassis.
     *
     * @param x   the position to move to in the x, in meters
     * @param y   the position to move to in the y, in meters
     * @param rot the angle to move to, in radians
     */
    // public void driveWithPID(double x, double y, double rot) {
    //     double xSpeed = xController.calculate(poseEstimator.getEstimatedPosition().getX(), x);
    //     double ySpeed = yController.calculate(poseEstimator.getEstimatedPosition().getY(), y);
    //     double rotRadians = rotationController.calculate(getYaw().getRadians(), rot);
    //     drive(xSpeed, ySpeed, rotRadians, true, false);
    // }

    /**
     * Updates the field relative position of the robot.
     */
    // public void updateOdometry() {
        // Updates pose based on encoders and gyro. NOTE: must use yaw directly from gyro!
        // poseEstimator.update(Rotation2d.fromDegrees(pigeon.getYaw()), getModulePositions());
    // }

    /**
     * Stops all swerve modules.
     */
    public void stop() {
        //Arrays.stream(modules).forEach(Module::stop);
    }


    // GETTERS AND SETTERS

    /**
     * Sets the desired states for all swerve modules.
     *
     * @param swerveModuleStates an array of module states to set swerve modules to. Order of the array matters here!
     */
    public void setModuleStates(SwerveModuleState[] swerveModuleStates, boolean isOpenLoop) {
        // makes sure speeds of modules don't exceed maximum allowed
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConst.kMaxSpeed);

        for (int i = 0; i < 4; i++) {
            //modules[i].setDesiredState(swerveModuleStates[i], isOpenLoop);
        }
    }

    /**
     * Sets the chassis speeds of the robot.
     *
     * @param chassisSpeeds the target chassis speeds
     * @param isOpenLoop    if open loop control should be used for the drive velocity
     */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds, boolean isOpenLoop) {
        if (Robot.isSimulation()) {
            // pigeon.getSimCollection().addHeading(
            //         +Units.radiansToDegrees(chassisSpeeds.omegaRadiansPerSecond * GlobalConst.LOOP_TIME));
        }
        SwerveModuleState[] swerveModuleStates = DriveConst.KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        setModuleStates(swerveModuleStates, isOpenLoop);
    }

    /**
     * Returns the angular rate from the pigeon.
     *
     * @param id 0 for x, 1 for y, 2 for z
     * @return the rate in rads/s from the pigeon
     */
    public double getAngularRate(int id) {
        // uses pass by reference and edits reference to array
        double[] rawGyros = new double[3];
        //pigeon.getRawGyro(rawGyros);

        // outputs in deg/s, so convert to rad/s
        return Units.degreesToRadians(rawGyros[id]);
    }

    /**
     * Gets an array of SwerveModulePositions, which store the distance travleled by the drive and the steer angle.
     *
     * @return an array of all swerve module positions
     */
    // public SwerveModulePosition[] getModulePositions() {
    //     return Arrays.stream(modules).map(Module::getPosition).toArray(SwerveModulePosition[]::new);
    // }

    /**
     * Enables or disables the state deadband for all swerve modules.
     * The state deadband determines if the robot will stop drive and steer motors when inputted drive velocity is low.
     * It should be enabled for all regular driving, to prevent releasing the controls from setting the angles.
     */
    public void setStateDeadband(boolean stateDeadBand) {
        //Arrays.stream(modules).forEach(module -> module.setStateDeadband(stateDeadBand));
    }


    /**
     * Calculates chassis speed of drivetrain using the current SwerveModuleStates
     * @return ChassisSpeeds object
     * This is often used as an input for other methods
     */
    // public ChassisSpeeds getChassisSpeeds() {
    //     // return DriveConst.KINEMATICS.toChassisSpeeds(
    //     //         Arrays.stream(modules).map(Module::getState).toArray(SwerveModuleState[]::new)
    //     // );
    // }

    /**
     * @return the yaw of the robot, aka heading, the direction it is facing
     */
    public Rotation2d getYaw() {
        Rotation2d test = new Rotation2d();
        return test;
    }

    public Module[] getModules(){
        return modules;
    }

    /**
     * Resets the yaw of the robot.
     *
     * @param rotation the new yaw angle as Rotation2d
     */
    public void setYaw(Rotation2d rotation) {
        resetOdometry(new Pose2d(getPose().getTranslation(), rotation));
    }

    /**
     * Resets the odometry to the given pose.
     *
     * @param pose the pose to reset to.
     */
    public void resetOdometry(Pose2d pose) {
        // NOTE: must use pigeon yaw for odometer!
       // poseEstimator.resetPosition(Rotation2d.fromDegrees(pigeon.getYaw()), getModulePositions(), pose);
    }

    /**
     * @return the pose of the robot as estimated by the odometry
     */
    public Pose2d getPose() {
    Pose2d test = new Pose2d();
     return test;
        //return poseEstimator.getEstimatedPosition();
    }

    /**
     * TODO: Comment
     */
    public void resetModulesToAbsolute() {
        //Arrays.stream(modules).forEach(Module::resetToAbsolute);
    }


    // getters for the PID Controllers
    public PIDController getXController() {
        return xController;
    }
    public PIDController getYController() {
        return yController;
    }
    public PIDController getRotationController() {
        return rotationController;
    }
    public Field2d getFeild(){
        return fieldDisplay;
    }

    public void driveVortex(double speedLeft, double speedRight) {
        // velocityController.setSetpoint(0.5);
        // leftVortex.set(-velocityController.calculate(leftVortex.getOutputCurrent()));
        // System.out.println(speedLeft);
        // rightVortex.set(velocityController.calculate(rightVortex.getOutputCurrent()));
        // System.out.println(speedRight);
        leftVortex.set(-0.5);
        rightVortex.set(0.5);


    }
    

}