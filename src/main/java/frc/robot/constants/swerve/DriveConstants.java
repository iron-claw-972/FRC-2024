package frc.robot.constants.swerve;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotId;
import frc.robot.constants.Constants;
import lib.COTSFalconSwerveConstants;

/**
 * GlobalConst are, by default, for the competition robot.
 * GlobalConst get changed if the RobotId detected is not the competition robot.
 */
public class DriveConstants {

    public static double kRobotWidthWithBumpers = Units.inchesToMeters(26.5 + 3.25 * 2);

    public static double kWheelRadius = Units.inchesToMeters(2);

    public static double kTrackWidth = Units.inchesToMeters(20.75);//22.75 swerve bot, 20.75 comp bot

    // use the gear ratios
    public static double kDriveGearRatio = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
    public static double kSteerGearRatio = 150.0 / 7.0;

    /* Drive Motor Characterization Values
     * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
    public static final double DRIVE_KS = 0.32 / 12.0; // 0.65559
    public static final double DRIVE_KV = 1.51 / 12.0; // 1.93074
    public static final double DRIVE_KA = 0.27 / 12.0; // 0.00214

    public static double kMaxSpeed = (Constants.MAX_RPM / 60.0) * kWheelRadius * 2 * Math.PI / kDriveGearRatio;

    // Need to convert tangential velocity (the m/s of the edge of the robot) to angular velocity (the radians/s of the robot)
    // To do so, divide by the radius. The radius is the diagonal of the square chassis, diagonal = sqrt(2) * side_length.
    public static double kMaxAngularSpeed = kMaxSpeed / ((kTrackWidth / 2) * Math.sqrt(2));

    // TODO: tune this better.
    public static double kMaxAngularAccel = 8 * 2 * Math.PI; // 8 rotations per second per second

    public static int kPigeon = 13;

    public static Rotation2d kStartingHeading = new Rotation2d();

    public static final Translation2d[] swerveModuleLocations = {
            new Translation2d(DriveConstants.kTrackWidth / 2, DriveConstants.kTrackWidth / 2),
            new Translation2d(DriveConstants.kTrackWidth / 2, -DriveConstants.kTrackWidth / 2),
            new Translation2d(-DriveConstants.kTrackWidth / 2, DriveConstants.kTrackWidth / 2),
            new Translation2d(-DriveConstants.kTrackWidth / 2, -DriveConstants.kTrackWidth / 2)
    };

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(swerveModuleLocations);

   public static int kDriveFrontLeft = 1;
    public static int kSteerFrontLeft = 2;
    public static int kEncoderFrontLeft = 3;
    public static double kSteerOffsetFrontLeft = 4.185;//0.058291152119637;//-3.060285486280918+Math.PI;

    public static int kDriveFrontRight = 10;
    public static int kSteerFrontRight = 11;
    public static int kEncoderFrontRight = 12;
    public static double kSteerOffsetFrontRight = 101.519+90;//-2.994324445724487;//-3.001994334161282;

    public static int kDriveBackLeft = 7;
    public static int kSteerBackLeft = 8;
    public static int kEncoderBackLeft = 9;
    public static double kSteerOffsetBackLeft = 38.997+180;//-2.540267050266266;//0.650406539440155+Math.PI;

    public static int kDriveBackRight = 4;
    public static int kSteerBackRight = 5;
    public static int kEncoderBackRight = 6;
    public static double kSteerOffsetBackRight = 242.847-90;//2.626169800758362;//2.771897681057453;

    // heading PID
    public static double kHeadingP = 4.6;
    public static double kHeadingD = 0;

    //translational PID
    public static double kTranslationalP = 0.25;
    public static double kTranslationalD = 0;//0.001

    //The PIDs for PathPlanner Command
    public static double kPathplannerHeadingP = 3.5;
    public static double kPathplannerHeadingD = 0;

    public static double kPathplannerTranslationalP = 6;
    public static double kPathplannerTranslationalD = 0;

    // CAN
    public static String kDriveMotorCAN = Constants.CANIVORE_CAN;
    public static String kSteerMotorCAN = Constants.CANIVORE_CAN;
    public static String kSteerEncoderCAN = Constants.CANIVORE_CAN;
    public static String kPigeonCAN = Constants.CANIVORE_CAN;


    public static final COTSFalconSwerveConstants kModuleConstants = COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.DriveGearRatios.SDSMK4i_L2);

    /* Swerve Current Limiting */
    public static final int kAngleContinuousCurrentLimit = 25;
    public static final int kAnglePeakCurrentLimit = 40;
    public static final double kAnglePeakCurrentDuration = 0.1;
    public static final boolean kAngleEnableCurrentLimit = true;

    public static final int kDriveContinuousCurrentLimit = 35;
    public static final int kDrivePeakCurrentLimit = 60;
    public static final double kDrivePeakCurrentDuration = 0.1;
    public static final boolean kDriveEnableCurrentLimit = true;

    /* Motor inversions */
    public static final boolean kDriveMotorInvert = true;//kModuleConstants.driveMotorInvert;
    public static final boolean kAngleMotorInvert = kModuleConstants.angleMotorInvert;

    /* Neutral Modes */
    public static final NeutralModeValue kDriveNeutralMode = NeutralModeValue.Brake;
    public static final NeutralModeValue kAngleNeutralMode = NeutralModeValue.Coast;

    /* Drive Motor PID Values */
    public static final double kDriveP = 0.05;
    public static final double kDriveI = 0.0;
    public static final double kDriveD = 0.0;
    public static final double kDriveF = 0.0;

    /* Ramp values for drive motors in open and closed loop driving. */
    // Open loop prevents throttle from changing too quickly.
    // It will limit it to time given (in seconds) to go from zero to full throttle.
    // A small open loop ramp (0.25) helps with tread wear, tipping, etc
    public static final double kOpenLoopRamp = 0.25;
    public static final double kClosedLoopRamp = 0.0;

    public static final double kWheelCircumference = kModuleConstants.wheelCircumference;

    /* Motor gear ratios */
    public static final double kAngleGearRatio = kModuleConstants.angleGearRatio;

    public static final boolean kInvertGyro = false; // Make sure gyro is CCW+ CW- // FIXME: Swerve

    public static final double kSlowDriveFactor = 0.2;
    public static final double kSlowRotFactor = 0.1;

    /**
     * Updates the constants if the RobotId is not the competition robot.
     */
    public static void update(RobotId robotId) {
        if (robotId == RobotId.SwerveTest) {

            kTrackWidth = Units.inchesToMeters(22.75); //22.75 swerve bot, 20.75 comp bot

            kPigeon = 13;

            kDriveFrontLeft = 1;
            kSteerFrontLeft = 2;
            kEncoderFrontLeft = 9;
            kSteerOffsetFrontLeft = -1.58;

            kDriveFrontRight = 7;
            kSteerFrontRight = 8;
            kEncoderFrontRight = 12;
            kSteerOffsetFrontRight = 1.935;

            kDriveBackLeft = 5;
            kSteerBackLeft = 6;
            kEncoderBackLeft = 11;
            kSteerOffsetBackLeft = -8;

            kDriveBackRight = 3;
            kSteerBackRight = 4;
            kEncoderBackRight = 10;
            kSteerOffsetBackRight = -0.383494421839714;

            // CAN
            kDriveMotorCAN = Constants.RIO_CAN;
        }
    }
}
