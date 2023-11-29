package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.Drivetrain;

/**
 * Stores information about an object detected by vision
 */
public class DetectedObject {
    private static Drivetrain drive;
    public final Pose2d pose;
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
        pose = new Pose2d();
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
        double horizontalDist = distance*Math.cos(yOffset+robotToCamera.getRotation().getY());
        double angle = robotToCamera.getRotation().getZ()-xOffset;
        double robotToCameraDist = robotToCamera.getTranslation().toTranslation2d().getNorm();
        double angle2 = 3*Math.PI/2-Math.atan2(robotToCamera.getX(), robotToCamera.getY())-angle;
        double robotToObjectDist = Math.sqrt(Math.pow(robotToCameraDist, 2)+Math.pow(horizontalDist, 2)-2*robotToCameraDist*horizontalDist*Math.cos(angle2));
        double angle3 = Math.atan2(robotToCamera.getY(), robotToCamera.getX())+Math.asin(horizontalDist*Math.sin(angle2)/robotToObjectDist);
        Translation2d translation = new Translation2d(robotToObjectDist*Math.cos(angle3), robotToObjectDist*Math.sin(angle3));
        translation = translation.rotateBy(drive.getYaw());
        translation = translation.plus(drive.getPose().getTranslation());
        pose = new Pose2d(translation, new Rotation2d());
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
        return drive.getPose().getTranslation().getDistance(pose.getTranslation());
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
