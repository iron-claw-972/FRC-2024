package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.Drivetrain;

public class DetectedObject {
    private static Drivetrain drive;
    public final Pose2d pose;
    public final ObjectType type;
    public enum ObjectType{CONE, CUBE, RED_ROBOT, BLUE_ROBOT, NONE};
    public static void setDrive(Drivetrain drive){
        DetectedObject.drive = drive;
    }
    public DetectedObject(){
        pose = new Pose2d();
        type = ObjectType.NONE;
    }
    public DetectedObject(double xOffset, double yOffset, double distance, ObjectType type, Transform3d robotToCamera){
        this.type = type;
        double horizontalDist = distance*Math.cos(yOffset+robotToCamera.getRotation().getY());
        double angle = robotToCamera.getRotation().getZ()-xOffset;
        double robotToCameraDist = robotToCamera.getTranslation().toTranslation2d().getNorm();
        double angle2 = 270-Math.atan2(robotToCamera.getX(), robotToCamera.getY())-angle;
        double robotToObjectDist = Math.sqrt(Math.pow(robotToCameraDist, 2)+Math.pow(horizontalDist, 2)-2*robotToCameraDist*horizontalDist*Math.cos(angle2));
        double angle3 = Math.atan2(robotToCamera.getY(), robotToCamera.getX())+Math.asin(horizontalDist*Math.sin(angle2)/robotToObjectDist);
        Translation2d translation = new Translation2d(robotToObjectDist*Math.cos(angle3), robotToObjectDist*Math.sin(angle3));
        translation = translation.rotateBy(drive.getYaw());
        translation = translation.plus(drive.getPose().getTranslation());
        pose = new Pose2d(translation, new Rotation2d());
    }
    public DetectedObject(double xOffset, double yOffset, double distance, String type, Transform3d robotToCamera){
        this(xOffset, yOffset, distance, 
            type.toLowerCase().equals("cone")?ObjectType.CONE:
            type.toLowerCase().equals("cube")?ObjectType.CUBE:
            type.toLowerCase().equals("red robot")?ObjectType.RED_ROBOT:
            type.toLowerCase().equals("blue robot")?ObjectType.BLUE_ROBOT:
            ObjectType.NONE, 
            robotToCamera
        );
    }
}
