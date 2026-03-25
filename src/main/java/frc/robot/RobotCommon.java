package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class RobotCommon {
    public static Pose2d currentRobotPose = Pose2d.kZero;
    public static Pose2d futureRobotPose = Pose2d.kZero; // 0.04 seconds in advance
    public static ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds();
    public static ChassisSpeeds robotRelativeSpeeds = new ChassisSpeeds();
    public static Rotation2d robotAngle = Rotation2d.kZero;

    public static boolean isRed = false;
    public static boolean isComp = false;
    public static boolean isRobotCalibrated = false;
}