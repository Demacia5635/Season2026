package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class RobotCommon {
    public enum Shifts{
        Auto, Transition, Active, Inactive, Endgame
    }

    public enum robotStates{
        ShootWithIntake, ShootWithoutIntake, DriveWhileIntake, Drive, PrepareClimb, Climb, GetOffClimb
    }
    public static robotStates currentState = robotStates.Drive;
    public static Pose2d currentRobotPose = Pose2d.kZero;
    public static Pose2d futureRobotPose = Pose2d.kZero; //0.04 seconds in advance
    public static ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds();
    public static ChassisSpeeds robotRelativeSpeeds = new ChassisSpeeds();
    public static Translation2d deliveryTarget = Translation2d.kZero;
    public static double targetAccuracy = 1; //0-1
    public static double distanceFromTarget = 0;
    public static double angleFormTarget = 0;
    public static double wantedTurretAngle = 0;
    public static boolean isRed = false;
    public static boolean isComp = false;
    public static boolean isRobotCalibrated = false;


}