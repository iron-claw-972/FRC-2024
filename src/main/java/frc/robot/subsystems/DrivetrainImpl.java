package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2.AxisDirection;
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
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.commands.test.CircleDrive;
import frc.robot.commands.test.TestDriveVelocity;
import frc.robot.commands.test.TestHeadingPID;
import frc.robot.commands.test.TestSteerAngle;
import frc.robot.constants.Constants;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.constants.swerve.ModuleConstants;
import frc.robot.util.LogManager;

/**
 * Represents a swerve drive style drivetrain.
 * <p>
 * Module IDs are:
 * 1: Front left
 * 2: Front right
 * 3: Back left
 * 4: Back right
 */
public class DrivetrainImpl extends Drivetrain {

    private final ShuffleboardTab swerveModulesTab;
    private final ShuffleboardTab drivetrainTab;

    // Odometry
    private final SwerveDrivePoseEstimator poseEstimator;

    // This is left intentionally public
    private final Module[] modules;

    private final WPI_Pigeon2 pigeon;

    // PID Controllers for chassis movement
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotationController;

    // Displays the field with the robots estimated pose on it
    private final Field2d fieldDisplay;

    //Shuffleboard
    private GenericEntry
            driveVelocityEntry,
            steerVelocityEntry,
            steerAngleEntry,
            driveStaticFeedforwardEntry,
            driveVelocityFeedforwardEntry,
            steerStaticFeedforwardEntry,
            steerVelocityFeedforwardEntry,
            xPosEntry,
            yPosEntry,
            headingEntry;

    private Double[] driveVelFeedForwardSaver = new Double[4];
    private Double[] driveStaticFeedForwardSaver = new Double[4];
    private Double[] steerVelFeedForwardSaver = new Double[4];
    private Double[] steerStaticFeedForwardSaver = new Double[4];

    private final SendableChooser<Module> moduleChooser = new SendableChooser<>();
    // modules needed to distinguish in chooser
    private Module prevModule;

    // If vision is enabled
    // Do not change this. Instead, change ENABLED in VisionConstants
    // This is used in some commands that need vision disabled
    private boolean visionEnabled = true;

    private int loggerStep = 0;


    /**
     * Creates a new Swerve Style Drivetrain.
     *
     * @param drivetrainTab    the shuffleboard tab to display drivetrain data on
     * @param swerveModulesTab the shuffleboard tab to display module data on
//     * @param vision           the vision
     */
    public DrivetrainImpl(ShuffleboardTab drivetrainTab, ShuffleboardTab swerveModulesTab) {

        this.drivetrainTab = drivetrainTab;
        this.swerveModulesTab = swerveModulesTab;

        pigeon = new WPI_Pigeon2(DriveConstants.kPigeon, DriveConstants.kPigeonCAN);
        pigeon.configFactoryDefault();
        // Our pigeon is mounted with y forward, and z upward
        pigeon.configMountPose(AxisDirection.PositiveY, AxisDirection.PositiveZ);

        if (RobotBase.isReal()) {
            modules = new Module[]{
                    new Module(ModuleConstants.FRONT_LEFT, swerveModulesTab),
                    new Module(ModuleConstants.FRONT_RIGHT, swerveModulesTab),
                    new Module(ModuleConstants.BACK_LEFT, swerveModulesTab),
                    new Module(ModuleConstants.BACK_RIGHT, swerveModulesTab),
                    };
        } else {
            modules = new ModuleSim[]{
                    new ModuleSim(ModuleConstants.FRONT_LEFT, swerveModulesTab),
                    new ModuleSim(ModuleConstants.FRONT_RIGHT, swerveModulesTab),
                    new ModuleSim(ModuleConstants.BACK_LEFT, swerveModulesTab),
                    new ModuleSim(ModuleConstants.BACK_RIGHT, swerveModulesTab),
                    };
        }

        prevModule = modules[0];

        /*
         * By pausing init for a second before setting module offsets, we avoid a bug
         * with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();

        pigeon.setYaw(DriveConstants.kStartingHeading.getDegrees());
        poseEstimator = new SwerveDrivePoseEstimator(
                DriveConstants.KINEMATICS,
                Rotation2d.fromDegrees(pigeon.getYaw()),
                getModulePositions(),
                new Pose2d() // initial Odometry Location
        );
//        poseEstimator.setVisionMeasurementStdDevs(VisionConstants.kBaseVisionPoseStdDevs);

        xController = new PIDController(DriveConstants.kTranslationalP, 0, DriveConstants.kTranslationalD);
        yController = new PIDController(DriveConstants.kTranslationalP, 0, DriveConstants.kTranslationalD);
        rotationController = new PIDController(DriveConstants.kHeadingP, 0, DriveConstants.kHeadingD);
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        rotationController.setTolerance(Units.degreesToRadians(0.25), Units.degreesToRadians(0.25));

        fieldDisplay = new Field2d();
        fieldDisplay.setRobotPose(getPose());

        setupDrivetrainShuffleboard();
        setupModulesShuffleboard();
    }

    @Override
    public void periodic() {
        updateDriveModuleFeedforwardShuffleboard();
        updateDriveModuleFeedforwardShuffleboard();

        updateOdometry();

        fieldDisplay.setRobotPose(getPose());

        if (Constants.DO_LOGGING) updateLogs();

    }
    // PIDs for Chassis movement

    public PIDController getXController() {
        return xController;
    }

    public PIDController getYController() {
        return yController;
    }

    public PIDController getRotationController() {
        return rotationController;
    }

    /**
     * @return chassis speed of swerve drive
     */
    public ChassisSpeeds getChassisSpeeds() {
        return DriveConstants.KINEMATICS.toChassisSpeeds(getModuleStates());
    }

    public ChassisSpeeds getFieldRelativeChassisSpeeds() {
        return ChassisSpeeds.fromFieldRelativeSpeeds(
                getChassisSpeeds(),
                getPose().getRotation()
                                                    );
    }

    public double getChassisSpeedsMagnitude() {
        return Math.hypot(
                getFieldRelativeChassisSpeeds().vxMetersPerSecond,
                getFieldRelativeChassisSpeeds().vyMetersPerSecond
                         );
    }

    public Rotation2d getFieldRelativeHeading() {
        return Rotation2d.fromRadians(Math.atan2(
                getFieldRelativeChassisSpeeds().vxMetersPerSecond,
                getFieldRelativeChassisSpeeds().vyMetersPerSecond
                                                ));
    }

    public Rotation2d getPitch() {
        return Rotation2d.fromDegrees(pigeon.getPitch());
    }

    public Rotation2d getRoll() {
        return Rotation2d.fromDegrees(pigeon.getRoll());
    }

    /**
     * @return the yaw of the robot, aka heading, the direction it is facing
     */
    public Rotation2d getYaw() {
        return poseEstimator.getEstimatedPosition().getRotation();
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
        poseEstimator.resetPosition(Rotation2d.fromDegrees(pigeon.getYaw()), getModulePositions(), pose);
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
        setChassisSpeeds((
                                 fieldRelative
                                         ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getYaw())
                                         : new ChassisSpeeds(xSpeed, ySpeed, rot)
                         ),
                         isOpenLoop
                        );
    }

    /**
     * Sets the desired states for all swerve modules.
     *
     * @param swerveModuleStates an array of module states to set swerve modules to. Order of the array matters here!
     */
    public void setModuleStates(SwerveModuleState[] swerveModuleStates, boolean isOpenLoop) {
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeed);
        for (int i = 0; i < 4; i++) {
            modules[i].setDesiredState(swerveModuleStates[i], isOpenLoop);
        }
    }

    /**
     * Sets the desired states for all swerve modules. Runs closed loop control. USe this function for pathplanner.
     *
     * @param swerveModuleStates an array of module states to set swerve modules to. Order of the array matters here!
     */
    public void setModuleStates(SwerveModuleState[] swerveModuleStates) {
        setModuleStates(swerveModuleStates, false);
    }

    /**
     * Gets the current robot pose from the pose estimator.
     */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Module[] getModules() {
        return modules;
    }

    /**
     * Enables or disables the state deadband for all swerve modules.
     * The state deadband determines if the robot will stop drive and steer motors when inputted drive velocity is low.
     * It should be enabled for all regular driving, to prevent releasing the controls from setting the angles.
     */
    public void enableStateDeadband(boolean stateDeadBand) {
        for (int i = 0; i < 4; i++) {
            modules[i].enableStateDeadband(stateDeadBand);
        }
    }

    /**
     * Gets an array of SwerveModulePositions, which store the distance travleled by the drive and the steer angle.
     *
     * @return an array of all swerve module positions
     */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (Module mod : modules) {
            positions[mod.getModuleIndex()] = mod.getPosition();
        }
        return positions;
    }

    /**
     * Gets an array of SwerveModuleStates, which store the drive velocity and steer angle
     *
     * @return an array of all swerve module positions
     */
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (Module mod : modules) {
            states[mod.getModuleIndex()] = mod.getState();
        }
        return states;
    }

    /**
     * Sets the chassis speeds of the robot.
     *
     * @param chassisSpeeds the target chassis speeds
     * @param isOpenLoop    if open loop control should be used for the drive velocity
     */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds, boolean isOpenLoop) {
        if (Robot.isSimulation()) {
            pigeon.getSimCollection().addHeading(
                    +Units.radiansToDegrees(chassisSpeeds.omegaRadiansPerSecond * Constants.LOOP_TIME));
        }
        SwerveModuleState[] swerveModuleStates = DriveConstants.KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        setModuleStates(swerveModuleStates, isOpenLoop);
    }

    public void resetModulesToAbsolute() {
        for (Module mod : modules) {
            mod.resetToAbsolute();
        }
    }

    /**
     * Updates the field relative position of the robot.
     */
    public void updateOdometry() {
        // Updates pose based on encoders and gyro. NOTE: must use yaw directly from gyro!
        poseEstimator.update(Rotation2d.fromDegrees(pigeon.getYaw()), getModulePositions());
        // Updates pose based on vision
//        if (RobotBase.isReal() && visionEnabled && VisionConstants.ENABLED) {
//
//            // An array list of poses returned by different cameras
//            ArrayList<EstimatedRobotPose> estimatedPoses = vision.getEstimatedPoses(poseEstimator.getEstimatedPosition());
//            // The current position as a translation
//            Translation2d currentEstimatedPoseTranslation = poseEstimator.getEstimatedPosition().getTranslation();
//            for (EstimatedRobotPose estimatedPose : estimatedPoses) {
//                // The position of the closest april tag as a translation
//                Translation2d closestTagPoseTranslation = null;
//                for (int j = 0; j < estimatedPose.targetsUsed.size(); j++) {
//                    // The position of the current april tag
//                    Pose3d currentTagPose = vision.getTagPose(estimatedPose.targetsUsed.get(j).getFiducialId());
//                    // If it can't find the april tag's pose, don't run the rest of the for loop for this tag
//                    if (currentTagPose == null) {
//                        continue;
//                    }
//                    Translation2d currentTagPoseTranslation = currentTagPose.toPose2d().getTranslation();
//
//                    // If the current april tag position is closer than the closest one, this makes makes it the closest
//                    if (closestTagPoseTranslation == null || currentEstimatedPoseTranslation.getDistance(currentTagPoseTranslation) < currentEstimatedPoseTranslation.getDistance(closestTagPoseTranslation)) {
//                        closestTagPoseTranslation = currentTagPoseTranslation;
//                    }
//                }
//
//                double visionFactor = (currentEstimatedPoseTranslation.getDistance(closestTagPoseTranslation) * VisionConstants.kVisionPoseStdDevFactor);
//
//                // Adds the vision measurement for this camera
//                poseEstimator.addVisionMeasurement(
//                        estimatedPose.estimatedPose.toPose2d(),
//                        estimatedPose.timestampSeconds,
//                        VisionConstants.kBaseVisionPoseStdDevs.plus(
//                                visionFactor
//                                                                   )
//                                                  );
//                if (Constants.DO_LOGGING) {
//                    LogManager.addDouble("Vision/ClosestTag Distance",
//                                         currentEstimatedPoseTranslation.getDistance(closestTagPoseTranslation)
//                                        );
//                }
//            }
//        }
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
        //SmartDashboard.putNumber("Heading PID Output", rot);
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
    public void runChassisPID(double x, double y, double rot) {
        double xSpeed = xController.calculate(poseEstimator.getEstimatedPosition().getX(), x);
        double ySpeed = yController.calculate(poseEstimator.getEstimatedPosition().getY(), y);
        double rotRadians = rotationController.calculate(getYaw().getRadians(), rot);
        drive(xSpeed, ySpeed, rotRadians, true, false);
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
        pigeon.getRawGyro(rawGyros);

        // outputs in deg/s, so convert to rad/s
        return Units.degreesToRadians(rawGyros[id]);
    }


    public void enableVision(boolean enabled) {
        visionEnabled = enabled;
    }

    /**
     * Sets the optimize state for all swerve modules.
     * Optimizing the state means the modules will not turn the steer motors more than 90 degrees for any one movement.
     */
    public void setAllOptimize(Boolean optimizeSate) {
        for (int i = 0; i < 4; i++) {
            modules[i].setOptimize(optimizeSate);
        }
    }

    /**
     * Stops all swerve modules.
     */
    public void stop() {
        for (int i = 0; i < 4; i++) {
            modules[i].stop();
        }
    }

    /**
     * Sets up feedforward savers.
     */
    private void setUpFeedforwardSavers() {
        driveStaticFeedForwardSaver = new Double[]{
                modules[0].getDriveFeedForwardKS(),
                modules[1].getDriveFeedForwardKS(),
                modules[2].getDriveFeedForwardKS(),
                modules[3].getDriveFeedForwardKS()
        };
        driveVelFeedForwardSaver = new Double[]{
                modules[0].getDriveFeedForwardKV(),
                modules[1].getDriveFeedForwardKV(),
                modules[2].getDriveFeedForwardKV(),
                modules[3].getDriveFeedForwardKV()
        };
        steerStaticFeedForwardSaver = new Double[]{
                modules[0].getSteerFeedForwardKS(),
                modules[1].getSteerFeedForwardKS(),
                modules[2].getSteerFeedForwardKS(),
                modules[3].getSteerFeedForwardKS()
        };
        steerVelFeedForwardSaver = new Double[]{
                modules[0].getSteerFeedForwardKV(),
                modules[1].getSteerFeedForwardKV(),
                modules[2].getSteerFeedForwardKV(),
                modules[3].getSteerFeedForwardKV()
        };
    }

    public Double[] getDriveStaticFeedforwardArray() {
        return driveStaticFeedForwardSaver;
    }

    public Double[] getDriveVelocityFeedforwardArray() {
        return driveVelFeedForwardSaver;
    }

    public Double[] getSteerStaticFeedforwardArray() {
        return steerStaticFeedForwardSaver;
    }

    public Double[] getSteerVelocityFeedforwardArray() {
        return steerVelFeedForwardSaver;
    }


    // BELOW IS TELEMETRY STUFF

    /**
     * Sets up the shuffleboard tab for the drivetrain.
     */
    private void setupDrivetrainShuffleboard() {

        drivetrainTab.add("Field", fieldDisplay);
        if (!Constants.USE_TELEMETRY) return;


        drivetrainTab.add("Balance PID", balanceController);

        // inputs
        headingEntry = drivetrainTab.add("Set Heading (-pi to pi)", 0).getEntry();
        xPosEntry = drivetrainTab.add("Input X pos(m)", 0).getEntry();
        yPosEntry = drivetrainTab.add("Input Y pos(m)", 0).getEntry();

        // add PID controllers
        drivetrainTab.add("xController", getXController());
        drivetrainTab.add("yController", getYController());
        drivetrainTab.add("rotationController", getRotationController());

        // add angles
        drivetrainTab.addNumber("Yaw (deg)", () -> getYaw().getDegrees());
        drivetrainTab.addNumber("estimated X", () -> poseEstimator.getEstimatedPosition().getX());
        drivetrainTab.addNumber("estimated Y", () -> poseEstimator.getEstimatedPosition().getY());
        drivetrainTab.addNumber("getPitch", () -> pigeon.getPitch());
        drivetrainTab.addNumber("getRoll", () -> pigeon.getRoll());
        drivetrainTab.addNumber("pigeon yaw", () -> pigeon.getYaw());

        drivetrainTab.addNumber("Gyro X", () -> getAngularRate(0));
        drivetrainTab.addNumber("Gyro Y", () -> getAngularRate(1));
        drivetrainTab.addNumber("Gyro Z", () -> getAngularRate(2));

        drivetrainTab.addNumber("Chassis Velocity", () -> getChassisSpeedsMagnitude());
    }

    /**
     * Sets up the shuffleboard tab for the swerve modules.
     */
    private void setupModulesShuffleboard() {
        if (Constants.USE_TELEMETRY) {

            moduleChooser.setDefaultOption("Front Left", modules[0]);
            moduleChooser.addOption("Front Right", modules[1]);
            moduleChooser.addOption("Back Left", modules[2]);
            moduleChooser.addOption("Back Right", modules[3]);

            setUpFeedforwardSavers();

            // inputs
            swerveModulesTab.add("Module Chooser", moduleChooser);
            driveVelocityEntry = swerveModulesTab.add("Set Drive Velocity", 0).getEntry();
            steerVelocityEntry = swerveModulesTab.add("Set Steer Velocity", 0).getEntry();
            steerAngleEntry = swerveModulesTab.add("Set Steer Angle", 0).getEntry();
            driveStaticFeedforwardEntry = swerveModulesTab.add(
                    "Drive kS FF",
                    driveStaticFeedForwardSaver[moduleChooser.getSelected().getModuleIndex()]
                                                              ).getEntry();

            driveVelocityFeedforwardEntry = swerveModulesTab.add(
                    "Drive kV FF",
                    driveVelFeedForwardSaver[moduleChooser.getSelected().getModuleIndex()]
                                                                ).getEntry();

            steerStaticFeedforwardEntry = swerveModulesTab.add(
                    "Steer kS FF",
                    steerStaticFeedForwardSaver[moduleChooser.getSelected().getModuleIndex()]
                                                              ).getEntry();

            steerVelocityFeedforwardEntry = swerveModulesTab.add(
                    "Steer kV FF",
                    steerVelFeedForwardSaver[moduleChooser.getSelected().getModuleIndex()]
                                                                ).getEntry();
        }
    }

    public double getRequestedHeading(double defaultValue) {
        if (!Constants.USE_TELEMETRY) return defaultValue;
        return headingEntry.getDouble(defaultValue);
    }

    public double getRequestedDriveVelocity(double defaultValue) {
        if (!Constants.USE_TELEMETRY) return defaultValue;
        return driveVelocityEntry.getDouble(defaultValue);
    }

    public double getRequestedSteerVelocity(double defaultValue) {
        if (!Constants.USE_TELEMETRY) return defaultValue;
        return steerVelocityEntry.getDouble(defaultValue);
    }

    public double getRequestedSteerAngle(double defaultValue) {
        if (!Constants.USE_TELEMETRY) return defaultValue;
        return steerAngleEntry.getDouble(defaultValue);
    }

    public double getRequestedXPos(double defaultValue) {
        if (!Constants.USE_TELEMETRY) return defaultValue;
        return xPosEntry.getDouble(defaultValue);
    }

    public double getRequestedYPos(double defaultValue) {
        if (!Constants.USE_TELEMETRY) return defaultValue;
        return yPosEntry.getDouble(defaultValue);
    }

    public void setDriveVelocityFeedforwardEntry(double value) {
        if (!Constants.USE_TELEMETRY) return;
        driveVelocityFeedforwardEntry.setDouble(value);
    }

    public void setDriveStaticFeedforwardEntry(double value) {
        if (!Constants.USE_TELEMETRY) return;
        driveStaticFeedforwardEntry.setDouble(value);
    }

    public void setSteerStaticFeedforwardEntry(double value) {
        if (!Constants.USE_TELEMETRY) return;
        steerStaticFeedforwardEntry.setDouble(value);
    }

    public void setSteerVelocityFeedforwardEntry(double value) {
        if (!Constants.USE_TELEMETRY) return;
        steerVelocityFeedforwardEntry.setDouble(value);
    }

    /**
     * Updates the drive module feedforward values on shuffleboard.
     */
    public void updateDriveModuleFeedforwardShuffleboard() {
        if (!Constants.USE_TELEMETRY) return;
        // revert to previous saved feed forward data if changed
        if (prevModule != moduleChooser.getSelected()) {
            driveStaticFeedforwardEntry.setDouble(
                    driveStaticFeedForwardSaver[moduleChooser.getSelected().getModuleIndex()]
                                                 );
            driveVelocityFeedforwardEntry.setDouble(
                    driveVelFeedForwardSaver[moduleChooser.getSelected().getModuleIndex()]
                                                   );
            prevModule = moduleChooser.getSelected();
        }

        // update saved feedforward data
        driveStaticFeedForwardSaver[moduleChooser.getSelected().getModuleIndex()] =
                driveStaticFeedforwardEntry.getDouble(0);
        driveVelFeedForwardSaver[moduleChooser.getSelected().getModuleIndex()] =
                driveVelocityFeedforwardEntry.getDouble(0);

        // to set all modules to same feedforward values if all
        // if (module.getSelected() == allModule) {
        //   for(int i = 0; i < 4; i++) {
        //     modules[i].setDriveFeedForwardValues(driveStaticFeedForwardSaver.get(module.getSelected()), driveVelFeedForwardSaver.get(module.getSelected()));
        //   }
        // }

        //set selected module
        moduleChooser.getSelected().setDriveFeedForwardValues(
                driveStaticFeedForwardSaver[moduleChooser.getSelected().getModuleIndex()],
                driveVelFeedForwardSaver[moduleChooser.getSelected().getModuleIndex()]
                                                             );
    }

    /**
     * Updates the steer module feedforward values on shuffleboard.
     */
    public void updateSteerModuleFeedforwardShuffleboard() {
        if (!Constants.USE_TELEMETRY) return;

        //revert to previous saved feed forward data if changed
        if (prevModule != moduleChooser.getSelected()) {
            steerStaticFeedforwardEntry.setDouble(
                    steerStaticFeedForwardSaver[moduleChooser.getSelected().getModuleIndex()]
                                                 );
            steerVelocityFeedforwardEntry.setDouble(
                    steerVelFeedForwardSaver[moduleChooser.getSelected().getModuleIndex()]
                                                   );
            prevModule = moduleChooser.getSelected();
        }

        // update saved feedforward data
        steerStaticFeedForwardSaver[moduleChooser.getSelected().getModuleIndex()] =
                steerStaticFeedforwardEntry.getDouble(0);
        steerVelFeedForwardSaver[moduleChooser.getSelected().getModuleIndex()] =
                steerVelocityFeedforwardEntry.getDouble(0);

        //to set all modules to same feedforward values if all
        // if (module.getSelected() == allModule) {
        //   for(int i = 0; i < 4; i++) {
        //     modules[i].setDriveFeedForwardValues(steerStaticFeedForwardSaver[module.getSelected().getId()], steerVelFeedForwardSaver[module.getSelected().getId()]);
        //   }
        // }

        //set selected module
        moduleChooser.getSelected().setDriveFeedForwardValues(
                steerStaticFeedForwardSaver[moduleChooser.getSelected().getModuleIndex()],
                steerVelFeedForwardSaver[moduleChooser.getSelected().getModuleIndex()]
                                                             );
    }

    public Module getModuleChoosen() {
        if (!Constants.USE_TELEMETRY) return modules[0];
        return moduleChooser.getSelected();
    }

    /**
     * Adds the test commands to shuffleboard so they can be run that way.
     */
    public void addTestCommands(ShuffleboardTab testTab, GenericEntry testEntry) {
        if (Constants.USE_TELEMETRY) {
            testTab.add("Circle Drive", new CircleDrive(this));
            testTab.add("Test Drive Velocity", new TestDriveVelocity(this, testEntry));
            testTab.add("Heading PID", new TestHeadingPID(this, testEntry));
            testTab.add("Steer angle", new TestSteerAngle(this, testEntry));
            testTab.add("Reset Pose", new InstantCommand(() -> {
                this.resetOdometry(
                        new Pose2d(
                                this.getRequestedXPos(0),
                                this.getRequestedYPos(0),
                                new Rotation2d(this.getRequestedHeading(0))
                        ));
            }
            ));
        }
    }

    public void updateLogs() {

        loggerStep++;
        if (loggerStep < 4) return;
        loggerStep = 0;

        double[] pose = {
                getPose().getX(),
                getPose().getY(),
                getPose().getRotation().getRadians()
        };
        LogManager.addDoubleArray("Swerve/Pose2d", pose);

        double[] actualStates = {
                modules[0].getAngle().getRadians(),
                modules[0].getState().speedMetersPerSecond,
                modules[1].getAngle().getRadians(),
                modules[1].getState().speedMetersPerSecond,
                modules[2].getAngle().getRadians(),
                modules[2].getState().speedMetersPerSecond,
                modules[3].getAngle().getRadians(),
                modules[3].getState().speedMetersPerSecond
        };
        LogManager.addDoubleArray("Swerve/actual swerve states", actualStates);

        double[] desiredStates = {
                modules[0].getDesiredAngle().getRadians(),
                modules[0].getDesiredVelocity(),
                modules[1].getDesiredAngle().getRadians(),
                modules[1].getDesiredVelocity(),
                modules[2].getDesiredAngle().getRadians(),
                modules[2].getDesiredVelocity(),
                modules[3].getDesiredAngle().getRadians(),
                modules[3].getDesiredVelocity()
        };
        LogManager.addDoubleArray("Swerve/desired swerve states", desiredStates);

        // double[] errorStates = {
        //   desiredStates[0] - actualStates[0],
        //   desiredStates[1] - actualStates[1],
        //   desiredStates[2] - actualStates[2],
        //   desiredStates[3] - actualStates[3],
        //   desiredStates[4] - actualStates[4],
        //   desiredStates[5] - actualStates[5],
        //   desiredStates[6] - actualStates[6],
        //   desiredStates[7] - actualStates[7]
        // };
        // LogManager.addDoubleArray("Swerve/error swerve states", errorStates);
    }
}