package frc.robot.controls;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.constants.globalConst;
import frc.robot.constants.swerve.DriveConstants;
import frc.robot.subsystems.Drivetrain.Drivetrain;
import frc.robot.util.DynamicSlewRateLimiter;
import frc.robot.util.MathUtils;

/**
 * Abstract class for different controller types.
 */
public abstract class BaseDriverConfig {

    private final Drivetrain drive;

    private final boolean shuffleboardUpdates;

    private final ShuffleboardTab controllerTab;
    private GenericEntry translationalSensitivityEntry, translationalExpoEntry, translationalDeadbandEntry, translationalSlewrateEntry;
    private GenericEntry rotationSensitivityEntry, rotationExpoEntry, rotationDeadbandEntry, rotationSlewrateEntry;
    private GenericEntry headingSensitivityEntry, headingExpoEntry, headingDeadbandEntry;

    // Some of these are not currently used, but we might want them later
    @SuppressWarnings("unused")
    private double translationalSensitivity = globalConst.TRANSLATIONAL_SENSITIVITY;
    @SuppressWarnings("unused")
    private double translationalExpo = globalConst.TRANSLATIONAL_EXPO;
    @SuppressWarnings("unused")
    private double translationalDeadband = globalConst.TRANSLATIONAL_DEADBAND;
    private double translationalSlewrate = globalConst.TRANSLATIONAL_SLEWRATE;

    @SuppressWarnings("unused")
    private double rotationSensitivity = globalConst.ROTATION_SENSITIVITY;
    @SuppressWarnings("unused")
    private double rotationExpo = globalConst.ROTATION_EXPO;
    @SuppressWarnings("unused")
    private double rotationDeadband = globalConst.ROTATION_DEADBAND;
    private double rotationSlewrate = globalConst.ROTATION_SLEWRATE;

    private double headingSensitivity = globalConst.HEADING_SENSITIVITY;
    private double headingExpo = globalConst.HEADING_EXPO;
    private double headingDeadband = globalConst.HEADING_DEADBAND;
    private double previousHeading = 0;

    @SuppressWarnings("unused")
    private final DynamicSlewRateLimiter xSpeedLimiter = new DynamicSlewRateLimiter(translationalSlewrate);
    @SuppressWarnings("unused")
    private final DynamicSlewRateLimiter ySpeedLimiter = new DynamicSlewRateLimiter(translationalSlewrate);
    @SuppressWarnings("unused")
    private final DynamicSlewRateLimiter rotLimiter = new DynamicSlewRateLimiter(rotationSlewrate);
    private final DynamicSlewRateLimiter headingLimiter = new DynamicSlewRateLimiter(headingSensitivity);

    /**
     * @param drive               the drivetrain instance
     * @param controllerTab       the shuffleboard controller tab
     * @param shuffleboardUpdates whether to update the shuffleboard
     */
    public BaseDriverConfig(Drivetrain drive, ShuffleboardTab controllerTab, boolean shuffleboardUpdates) {
        headingLimiter.setContinuousLimits(-Math.PI, Math.PI);
        headingLimiter.enableContinuous(true);
        this.controllerTab = controllerTab;
        this.shuffleboardUpdates = shuffleboardUpdates;
        this.drive = drive;
    }

    public double getForwardTranslation() {
        return MathUtils.expoMS(MathUtil.applyDeadband(getRawForwardTranslation(), globalConst.DEADBAND), 2) * DriveConstants.kMaxSpeed * 1;
    }

    public double getSideTranslation() {
        return MathUtils.expoMS(MathUtil.applyDeadband(getRawSideTranslation(), globalConst.DEADBAND), 2) * DriveConstants.kMaxSpeed * 1;
    }

    public double getRotation() {
        return MathUtils.expoMS(MathUtil.applyDeadband(getRawRotation(), globalConst.DEADBAND), 2) * DriveConstants.kMaxAngularSpeed * 1;
    }

    public double getHeading() {
        if (getRawHeadingMagnitude() <= headingDeadband) return headingLimiter.calculate(previousHeading, 1e-6);
        previousHeading = headingLimiter.calculate(getRawHeadingAngle(), MathUtils.expoMS(getRawHeadingMagnitude(), headingExpo) * headingSensitivity);
        return previousHeading;
    }

    protected Drivetrain getDrivetrain() {
        return drive;
    }


    /**
     * Sets up shuffleboard values for the controller.
     */
    public void setupShuffleboard() {
        if (!shuffleboardUpdates) return;

        translationalSensitivityEntry = controllerTab.add("translationalSensitivity", globalConst.TRANSLATIONAL_SENSITIVITY).getEntry();
        translationalExpoEntry = controllerTab.add("translationalExpo", globalConst.TRANSLATIONAL_EXPO).getEntry();
        translationalDeadbandEntry = controllerTab.add("translationalDeadband", globalConst.TRANSLATIONAL_DEADBAND).getEntry();
        translationalSlewrateEntry = controllerTab.add("translationalSlewrate", globalConst.TRANSLATIONAL_SLEWRATE).getEntry();
        rotationSensitivityEntry = controllerTab.add("rotationSensitivity", globalConst.ROTATION_SENSITIVITY).getEntry();
        rotationExpoEntry = controllerTab.add("rotationExpo", globalConst.ROTATION_EXPO).getEntry();
        rotationDeadbandEntry = controllerTab.add("rotationDeadband", globalConst.ROTATION_DEADBAND).getEntry();
        rotationSlewrateEntry = controllerTab.add("rotationSlewrate", globalConst.ROTATION_SLEWRATE).getEntry();
        headingSensitivityEntry = controllerTab.add("headingSensitivity", globalConst.HEADING_SENSITIVITY).getEntry();
        headingExpoEntry = controllerTab.add("headingExpo", globalConst.HEADING_EXPO).getEntry();
        headingDeadbandEntry = controllerTab.add("headingDeadband", globalConst.HEADING_DEADBAND).getEntry();
    }

    /**
     * Updates the controller settings from shuffleboard.
     */
    public void updateSettings() { //updates the shuffleboard data
        if (!shuffleboardUpdates) return;

        translationalSensitivity = translationalSensitivityEntry.getDouble(globalConst.TRANSLATIONAL_SENSITIVITY);
        translationalExpo = translationalExpoEntry.getDouble(globalConst.TRANSLATIONAL_EXPO);
        translationalDeadband = translationalDeadbandEntry.getDouble(globalConst.TRANSLATIONAL_DEADBAND);
        translationalSlewrate = translationalSlewrateEntry.getDouble(globalConst.TRANSLATIONAL_SLEWRATE);

        rotationSensitivity = rotationSensitivityEntry.getDouble(globalConst.ROTATION_SENSITIVITY);
        rotationExpo = rotationExpoEntry.getDouble(globalConst.ROTATION_EXPO);
        rotationDeadband = rotationDeadbandEntry.getDouble(globalConst.ROTATION_DEADBAND);
        rotationSlewrate = rotationSlewrateEntry.getDouble(globalConst.ROTATION_SLEWRATE);

        headingSensitivity = headingSensitivityEntry.getDouble(globalConst.HEADING_SENSITIVITY);
        headingExpo = headingExpoEntry.getDouble(globalConst.HEADING_EXPO);
        headingDeadband = headingDeadbandEntry.getDouble(globalConst.HEADING_DEADBAND);
    }

    /**
     * Configures the controls for the controller.
     */
    public abstract void configureControls();

    public abstract double getRawSideTranslation();

    public abstract double getRawForwardTranslation();

    public abstract double getRawRotation();

    public abstract double getRawHeadingAngle();

    public abstract double getRawHeadingMagnitude();

    public abstract boolean getIsSlowMode();

    public abstract boolean getIsAlign();
}