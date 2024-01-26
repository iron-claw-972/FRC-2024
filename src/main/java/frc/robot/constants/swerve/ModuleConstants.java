package frc.robot.constants.swerve;

/**
 * Container class for module constants, defined using constants from {@link DriveConstants}
 * .
 *
 * @see DriveConstants
 */
public enum ModuleConstants {

    FRONT_LEFT(
            DriveConstants.kDriveFrontLeft,
            DriveConstants.kSteerFrontLeft,
            DriveConstants.kEncoderFrontLeft,
            DriveConstants.kSteerOffsetFrontLeft,
            ModuleType.FRONT_LEFT
    ),
    FRONT_RIGHT(
            DriveConstants.kDriveFrontRight,
            DriveConstants.kSteerFrontRight,
            DriveConstants.kEncoderFrontRight,
            DriveConstants.kSteerOffsetFrontRight,
            ModuleType.FRONT_RIGHT
    ),
    BACK_LEFT(
            DriveConstants.kDriveBackLeft,
            DriveConstants.kSteerBackLeft,
            DriveConstants.kEncoderBackLeft,
            DriveConstants.kSteerOffsetBackLeft,
            ModuleType.BACK_LEFT
    ),
    BACK_RIGHT(
            DriveConstants.kDriveBackRight,
            DriveConstants.kSteerBackRight,
            DriveConstants.kEncoderBackRight,
            DriveConstants.kSteerOffsetBackRight,
            ModuleType.BACK_RIGHT
    ),

    NONE(0, 0, 0, 0.0, ModuleType.NONE);

    private final int drivePort;
    private final int steerPort;
    private final int encoderPort;
    private final double steerOffset;
    private final ModuleType type;

    ModuleConstants(
            int drivePort,
            int steerPort,
            int encoderPort,
            double steerOffset,
            ModuleType type
                   ) {
        this.drivePort = drivePort;
        this.steerPort = steerPort;
        this.encoderPort = encoderPort;
        this.steerOffset = steerOffset;
        this.type = type;
    }

    public int getDrivePort() {
        return drivePort;
    }

    public int getSteerPort() {
        return steerPort;
    }

    public int getEncoderPort() {
        return encoderPort;
    }

    public double getSteerOffset() {
        return steerOffset;
    }

    public ModuleType getType() {
        return type;
    }

}