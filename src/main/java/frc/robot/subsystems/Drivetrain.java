package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.constants.miscConstants.VisionConstants;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.constants.swerve.ModuleConstants;
import frc.robot.subsystems.module.Module;
import frc.robot.subsystems.module.ModuleSim;
import frc.robot.util.Vision;

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
    private final SwerveDrivePoseEstimator poseEstimator;

    // Vision
    private final Vision vision;

    private final Pigeon2 pigeon;

    // PID Controllers for chassis movement
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotationController;

    // If vision is enabled for drivetrain odometry updating
    // DO NOT CHANGE THIS HERE TO DISABLE VISION, change VisionConstants.ENABLED instead
    private boolean visionEnabled = true;

    // If the robot should aim at the speaker
    private boolean isAlign = false;

    // If the shoot command is currently running, and the swerve angular speed should be locked
    private boolean isShooting = false;
    // Radians in which the shoot command is requesting
    public double shootAngle = 0;

    /**
     * Creates a new Swerve Style Drivetrain.
     */
    public Drivetrain(Vision vision) {
        this.vision = vision;

        modules = new Module[4];

        ModuleConstants[] constants = Arrays.copyOfRange(ModuleConstants.values(), 0, 4);
        
        if(RobotBase.isReal()){
            Arrays.stream(constants).forEach(moduleConstants -> {
                modules[moduleConstants.ordinal()] = new Module(moduleConstants);
            });
        }else{
            Arrays.stream(constants).forEach(moduleConstants -> {
                modules[moduleConstants.ordinal()] = new ModuleSim(moduleConstants);
            });
        }

        // The Pigeon is a gyroscope and implements WPILib's Gyro interface
        pigeon = new Pigeon2(DriveConstants.kPigeon, DriveConstants.kPigeonCAN);
        pigeon.getConfigurator().apply(new Pigeon2Configuration());
        // Our pigeon is mounted with y forward, and z upward
        MountPoseConfigs mountPoseConfigs = new MountPoseConfigs();
        mountPoseConfigs.deserialize("");
        pigeon.getConfigurator().apply(new MountPoseConfigs().withMountPosePitch(0).withMountPoseRoll(0).withMountPoseYaw(90));

        /*
         * By pausing init for a second before setting module offsets, we avoid a bug
         * with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();
        
        // initial Odometry Location
        pigeon.setYaw(DriveConstants.kStartingHeading.getDegrees());
        poseEstimator = new SwerveDrivePoseEstimator(
                DriveConstants.KINEMATICS,
                Rotation2d.fromDegrees(pigeon.getYaw().getValue()),
                getModulePositions(),
                new Pose2d() 
        );
       poseEstimator.setVisionMeasurementStdDevs(VisionConstants.VISION_STD_DEVS);
        
        // initialize PID controllers
        xController = new PIDController(DriveConstants.kTranslationalP, 0, DriveConstants.kTranslationalD);
        yController = new PIDController(DriveConstants.kTranslationalP, 0, DriveConstants.kTranslationalD);
        rotationController = new PIDController(DriveConstants.kHeadingP, 0, DriveConstants.kHeadingD);
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        rotationController.setTolerance(Units.degreesToRadians(0.25), Units.degreesToRadians(0.25));

    }

    @Override
    public void periodic() {
        updateOdometry();
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
    public void driveWithPID(double x, double y, double rot) {
        double xSpeed = xController.calculate(poseEstimator.getEstimatedPosition().getX(), x);
        double ySpeed = yController.calculate(poseEstimator.getEstimatedPosition().getY(), y);
        double rotRadians = rotationController.calculate(getYaw().getRadians(), rot);
        drive(xSpeed, ySpeed, rotRadians, true, false);
    }

    /**
     * Updates the field relative position of the robot.
     */
    public void updateOdometry() {
        // Updates pose based on encoders and gyro. NOTE: must use yaw directly from gyro!
        poseEstimator.update(Rotation2d.fromDegrees(pigeon.getYaw().getValue()), getModulePositions());

        if(RobotBase.isReal() && VisionConstants.ENABLED && visionEnabled){
            vision.updateOdometry(poseEstimator);
        }
    }

    /**
     * Stops all swerve modules.
     */
    public void stop() {
        Arrays.stream(modules).forEach(Module::stop);
    }


    // GETTERS AND SETTERS

    /**
     * Sets the desired states for all swerve modules.
     *
     * @param swerveModuleStates an array of module states to set swerve modules to. Order of the array matters here!
     */
    public void setModuleStates(SwerveModuleState[] swerveModuleStates, boolean isOpenLoop) {
        // makes sure speeds of modules don't exceed maximum allowed
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeed);

        for (int i = 0; i < 4; i++) {
            modules[i].setDesiredState(swerveModuleStates[i], isOpenLoop);
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
            pigeon.getSimState().addYaw(
                    +Units.radiansToDegrees(chassisSpeeds.omegaRadiansPerSecond * Constants.LOOP_TIME));
        }
        SwerveModuleState[] swerveModuleStates = DriveConstants.KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        setModuleStates(swerveModuleStates, isOpenLoop);
    }

    /**
     * Returns the angular rate from the pigeon.
     *
     * @param id 0 for x, 1 for y, 2 for z
     * @return the rate in rads/s from the pigeon
     */
    public double getAngularRate(int id) {
        double speed = 0;
        switch(id){
            case 0:
                speed = pigeon.getAngularVelocityXWorld().getValue();
                break;
            case 1:
                speed = pigeon.getAngularVelocityYWorld().getValue();
                break;
            case 2:
                speed = pigeon.getAngularVelocityZWorld().getValue();
                break;
        }
        // outputs in deg/s, so convert to rad/s
        return Units.degreesToRadians(speed);
    }

    /**
     * Gets an array of SwerveModulePositions, which store the distance travleled by the drive and the steer angle.
     *
     * @return an array of all swerve module positions
     */
    public SwerveModulePosition[] getModulePositions() {
        return Arrays.stream(modules).map(Module::getPosition).toArray(SwerveModulePosition[]::new);
    }

    /**
     * Enables or disables the state deadband for all swerve modules.
     * The state deadband determines if the robot will stop drive and steer motors when inputted drive velocity is low.
     * It should be enabled for all regular driving, to prevent releasing the controls from setting the angles.
     */
    public void setStateDeadband(boolean stateDeadBand) {
        Arrays.stream(modules).forEach(module -> module.setStateDeadband(stateDeadBand));
    }

    public void setVisionEnabled(boolean enabled){
        visionEnabled = enabled;
    }

    public void setIsAlign(boolean isAlign){
        this.isAlign = isAlign;
    }
    public boolean getIsAlign(){
        return isAlign;
    }

    /**
     * Calculates chassis speed of drivetrain using the current SwerveModuleStates
     * @return ChassisSpeeds object
     * This is often used as an input for other methods
     */
    public ChassisSpeeds getChassisSpeeds() {
        return DriveConstants.KINEMATICS.toChassisSpeeds(
                Arrays.stream(modules).map(Module::getState).toArray(SwerveModuleState[]::new)
        );
    }

    /**
     * @return the yaw of the robot, aka heading, the direction it is facing
     */
    public Rotation2d getYaw() {
        return poseEstimator.getEstimatedPosition().getRotation();
    }

    /**
     * @return an array of modules
     */
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
        poseEstimator.resetPosition(Rotation2d.fromDegrees(pigeon.getYaw().getValue()), getModulePositions(), pose);
    }

    /**
     * @return the pose of the robot as estimated by the odometry
     */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * TODO: Comment
     */
    public void resetModulesToAbsolute() {
        Arrays.stream(modules).forEach(Module::resetToAbsolute);
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
}