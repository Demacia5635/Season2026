
package frc.robot.Shooter.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.RobotContainer;

public class ShooterUtils {

    public static Translation3d GetChassisVelAsVector(){
      return new Translation3d(RobotContainer.intsnace.chassis.getRobotRelVelocities().vxMetersPerSecond, RobotContainer.intsnace.chassis.getRobotRelVelocities().vyMetersPerSecond, 0);
    }

    public static double distance(Pose2d from, Pose2d to) {
        return from.getTranslation().getDistance(to.getTranslation());
    }

    public static double angle(Pose2d from, Pose2d to){
        return  MathUtil.angleModulus(to.getTranslation().minus(from.getTranslation()).getAngle().getRadians());
    }

    public static Pose2d computeFuturePosition(ChassisSpeeds speeds, Pose2d currentPose, double dtSeconds) {
        Pose2d poseAtTime = new Pose2d(
            currentPose.getX() + (speeds.vxMetersPerSecond * dtSeconds),
            currentPose.getY() + (speeds.vyMetersPerSecond * dtSeconds),
            currentPose.getRotation().plus(new Rotation2d((speeds.omegaRadiansPerSecond * dtSeconds))));
        return poseAtTime;
    }

} 