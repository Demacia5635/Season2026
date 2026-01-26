
package frc.robot.Shooter.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.RobotContainer;

<<<<<<< HEAD
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
=======
public class ShooterUtils{
  /** Creates a new shooterUtilse. */

  static Shooter shooter;
  static Chassis chassis;

  public ShooterUtils(Chassis chassis) {
    this.chassis = chassis;
  }

  public static Translation3d GetChassisVelAsVector() {
    return new Translation3d(chassis.getRobotRelVelocities().vxMetersPerSecond,
        chassis.getRobotRelVelocities().vyMetersPerSecond, 0);
  }

  public static double distensFromToPose2dPoint(Pose2d from, Pose2d to) {
    return from.getTranslation().getDistance(to.getTranslation());
  }

  public static double angle_betuenTowPose2d(Pose2d from, Pose2d to) {
    return to.getTranslation().minus(from.getTranslation()).getAngle().getRadians();
  }

  public static Pose2d computeFuturePosition(ChassisSpeeds speeds, Pose2d currentPose, double dtSeconds) {
    Pose2d poseAtTime = currentPose.exp(new Twist2d(
        (speeds.vxMetersPerSecond * dtSeconds),
        (speeds.vyMetersPerSecond * dtSeconds),
        speeds.omegaRadiansPerSecond * dtSeconds));
    return poseAtTime;
  }

}
>>>>>>> origin/Shooter
