package frc.robot.constants.swerve;

/**
 * Container class for module constants, defined using constants from {@link DriveConst}
 * .
 *
 * @see DriveConst
 */
public enum ModuleConst {

    FRONT_LEFT(
            DriveConst.kDriveFrontLeft,
            DriveConst.kSteerFrontLeft,
            DriveConst.kEncoderFrontLeft,
            DriveConst.kSteerOffsetFrontLeft,
            ModuleType.FRONT_LEFT
    ),
    FRONT_RIGHT(
            DriveConst.kDriveFrontRight,
            DriveConst.kSteerFrontRight,
            DriveConst.kEncoderFrontRight,
            DriveConst.kSteerOffsetFrontRight,
            ModuleType.FRONT_RIGHT
    ),
    BACK_LEFT(
            DriveConst.kDriveBackLeft,
            DriveConst.kSteerBackLeft,
            DriveConst.kEncoderBackLeft,
            DriveConst.kSteerOffsetBackLeft,
            ModuleType.BACK_LEFT
    ),
    BACK_RIGHT(
            DriveConst.kDriveBackRight,
            DriveConst.kSteerBackRight,
            DriveConst.kEncoderBackRight,
            DriveConst.kSteerOffsetBackRight,
            ModuleType.BACK_RIGHT
    ),

    NONE(0, 0, 0, 0.0, ModuleType.NONE);

    private final int drivePort;
    private final int steerPort;
    private final int encoderPort;
    private final double steerOffset;
    private final ModuleType type;

    ModuleConst(
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