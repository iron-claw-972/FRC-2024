package frc.robot.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.Drivetrain;

/**
 * Stores information about an object detected by vision
 */
public class DetectedObject {
    private static Drivetrain drive;
    public final Pose3d pose;
    public final ObjectType type;
    public enum ObjectType{CONE, CUBE, RED_ROBOT, BLUE_ROBOT, NONE};

    /**
     * Sets the drivetrain to use for pose calculations
     * @param drive The drivetrain
     */
    public static void setDrive(Drivetrain drive){
        DetectedObject.drive = drive;
    }

    /**
     * Creates a default DetectedObject with default attributes
     */
    public DetectedObject(){
        pose = new Pose3d();
        type = ObjectType.NONE;
    }
    /**
     * Creates a new DetectedObject
     * @param xOffset The x offset from the camera to the object in radians
     * @param yOffset The y offset form the camera to the object in radians
     * @param distance The distance from the camera to the object in meters
     * @param type What type of object it is
     * @param robotToCamera The transformation form the robot to the camera
     */
    public DetectedObject(double xOffset, double yOffset, double distance, ObjectType type, Transform3d robotToCamera){
        this.type = type;
        Translation3d translation = new Translation3d(distance, new Rotation3d(0, yOffset, -xOffset));
        translation.rotateBy(robotToCamera.getRotation());
        translation.plus(robotToCamera.getTranslation());
        translation.rotateBy(new Rotation3d(
            drive.getRoll().getRadians(),
            drive.getPitch().getRadians(),
            drive.getYaw().getRadians()
        ));
        Translation2d drivePose = drive.getPose().getTranslation();
        translation.plus(new Translation3d(
            drivePose.getX(),
            drivePose.getY(),
            0
        ));
        pose = new Pose3d(translation, new Rotation3d());
    }
    /**
     * Creates a new DetectedObject
     * @param xOffset The x offset from the camera to the object in radians
     * @param yOffset The y offset form the camera to the object in radians
     * @param distance The distance from the camera to the object in meters
     * @param type What type of object it is
     * @param robotToCamera The transformation form the robot to the camera
     */
    public DetectedObject(double xOffset, double yOffset, double distance, String type, Transform3d robotToCamera){
        this(xOffset, yOffset, distance, 
            type==null?ObjectType.NONE:
            type.toLowerCase().equals("cone")?ObjectType.CONE:
            type.toLowerCase().equals("cube")?ObjectType.CUBE:
            type.toLowerCase().equals("red robot")?ObjectType.RED_ROBOT:
            type.toLowerCase().equals("blue robot")?ObjectType.BLUE_ROBOT:
            ObjectType.NONE, 
            robotToCamera
        );
    }

    /**
     * Returns if the object is a game piece
     * @return True if the object is a cone or cube, false otherwise
     */
    public boolean isGamePiece(){
        return type==ObjectType.CONE || type==ObjectType.CUBE;
    }
    /**
     * Returns if the object is a robot
     * @return True if the object is a red or blue robot, false otherwise
     */
    public boolean isRobot(){
        return type==ObjectType.RED_ROBOT || type==ObjectType.BLUE_ROBOT;
    }
    /**
     * Returns if the object is a robot on the same alliance
     * @return If the object is a robot on the same alliance
     */
    public boolean isSameAllianceRobot(){
        return type == (DriverStation.getAlliance()==Alliance.Red?ObjectType.RED_ROBOT:ObjectType.BLUE_ROBOT);
    }
    /**
     * Returns if the object is a robot on the other alliance
     * @return If the object is a robot on the other alliance
     */
    public boolean isOtherAllianceRobot(){
        return type == (DriverStation.getAlliance()==Alliance.Red?ObjectType.BLUE_ROBOT:ObjectType.RED_ROBOT);
    }

    /**
     * Gets the distance from the center of the robot to the object
     * @return The distance in meters
     */
    public double getDistance(){
        return drive.getPose().getTranslation().getDistance(pose.getTranslation().toTranslation2d());
    }
    /**
     * Gets the field relative angle from the robot to the object
     * @return The angle in radians
     */
    public double getAngle(){
        return Math.atan2(pose.getY()-drive.getPose().getY(), pose.getX()-drive.getPose().getX());
    }

    /**
     * Gets the angle relative to the robot (0 is in front, positive counterclockwise)
     * @return The relative angle in radians
     */
    public double getRelativeAngle(){
        double angle = getAngle()-drive.getYaw().getRadians();
        if(angle > Math.PI){
            angle -= Math.PI*2;
        }else if(angle < -Math.PI){
            angle += Math.PI*2;
        }
        return angle;
    }
}
