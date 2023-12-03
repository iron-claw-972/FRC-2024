package frc.robot.controls;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.constants.GlobalConst;
import frc.robot.constants.swerve.DriveConst;
import frc.robot.subsystems.drive.Drivetrain;
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
    private double translationalSensitivity = GlobalConst.TRANSLATIONAL_SENSITIVITY;
    @SuppressWarnings("unused")
    private double translationalExpo = GlobalConst.TRANSLATIONAL_EXPO;
    @SuppressWarnings("unused")
    private double translationalDeadband = GlobalConst.TRANSLATIONAL_DEADBAND;
    private double translationalSlewrate = GlobalConst.TRANSLATIONAL_SLEWRATE;

    @SuppressWarnings("unused")
    private double rotationSensitivity = GlobalConst.ROTATION_SENSITIVITY;
    @SuppressWarnings("unused")
    private double rotationExpo = GlobalConst.ROTATION_EXPO;
    @SuppressWarnings("unused")
    private double rotationDeadband = GlobalConst.ROTATION_DEADBAND;
    private double rotationSlewrate = GlobalConst.ROTATION_SLEWRATE;

    private double headingSensitivity = GlobalConst.HEADING_SENSITIVITY;
    private double headingExpo = GlobalConst.HEADING_EXPO;
    private double headingDeadband = GlobalConst.HEADING_DEADBAND;
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
        return -MathUtils.expoMS(MathUtil.applyDeadband(getRawForwardTranslation(), GlobalConst.DEADBAND), 2) * DriveConst.kMaxSpeed * 1;
    }

    public double getSideTranslation() {
        return -MathUtils.expoMS(MathUtil.applyDeadband(getRawSideTranslation(), GlobalConst.DEADBAND), 2) * DriveConst.kMaxSpeed * 1;
    }

    public double getRotation() {
        return -MathUtils.expoMS(MathUtil.applyDeadband(getRawRotation(), GlobalConst.DEADBAND), 2) * DriveConst.kMaxAngularSpeed * 1;
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

        translationalSensitivityEntry = controllerTab.add("translationalSensitivity", GlobalConst.TRANSLATIONAL_SENSITIVITY).getEntry();
        translationalExpoEntry = controllerTab.add("translationalExpo", GlobalConst.TRANSLATIONAL_EXPO).getEntry();
        translationalDeadbandEntry = controllerTab.add("translationalDeadband", GlobalConst.TRANSLATIONAL_DEADBAND).getEntry();
        translationalSlewrateEntry = controllerTab.add("translationalSlewrate", GlobalConst.TRANSLATIONAL_SLEWRATE).getEntry();
        rotationSensitivityEntry = controllerTab.add("rotationSensitivity", GlobalConst.ROTATION_SENSITIVITY).getEntry();
        rotationExpoEntry = controllerTab.add("rotationExpo", GlobalConst.ROTATION_EXPO).getEntry();
        rotationDeadbandEntry = controllerTab.add("rotationDeadband", GlobalConst.ROTATION_DEADBAND).getEntry();
        rotationSlewrateEntry = controllerTab.add("rotationSlewrate", GlobalConst.ROTATION_SLEWRATE).getEntry();
        headingSensitivityEntry = controllerTab.add("headingSensitivity", GlobalConst.HEADING_SENSITIVITY).getEntry();
        headingExpoEntry = controllerTab.add("headingExpo", GlobalConst.HEADING_EXPO).getEntry();
        headingDeadbandEntry = controllerTab.add("headingDeadband", GlobalConst.HEADING_DEADBAND).getEntry();
    }

    /**
     * Updates the controller settings from shuffleboard.
     */
    public void updateSettings() { //updates the shuffleboard data
        if (!shuffleboardUpdates) return;

        translationalSensitivity = translationalSensitivityEntry.getDouble(GlobalConst.TRANSLATIONAL_SENSITIVITY);
        translationalExpo = translationalExpoEntry.getDouble(GlobalConst.TRANSLATIONAL_EXPO);
        translationalDeadband = translationalDeadbandEntry.getDouble(GlobalConst.TRANSLATIONAL_DEADBAND);
        translationalSlewrate = translationalSlewrateEntry.getDouble(GlobalConst.TRANSLATIONAL_SLEWRATE);

        rotationSensitivity = rotationSensitivityEntry.getDouble(GlobalConst.ROTATION_SENSITIVITY);
        rotationExpo = rotationExpoEntry.getDouble(GlobalConst.ROTATION_EXPO);
        rotationDeadband = rotationDeadbandEntry.getDouble(GlobalConst.ROTATION_DEADBAND);
        rotationSlewrate = rotationSlewrateEntry.getDouble(GlobalConst.ROTATION_SLEWRATE);

        headingSensitivity = headingSensitivityEntry.getDouble(GlobalConst.HEADING_SENSITIVITY);
        headingExpo = headingExpoEntry.getDouble(GlobalConst.HEADING_EXPO);
        headingDeadband = headingDeadbandEntry.getDouble(GlobalConst.HEADING_DEADBAND);
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