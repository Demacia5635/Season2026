
package frc.robot.Shooter.utils;

import frc.demacia.utils.geometry.ChassisSpeedsDemacia;
import frc.demacia.utils.geometry.Pose2dDemacia;
import frc.demacia.utils.geometry.Rotation2dDemacia;
import frc.demacia.utils.geometry.Translation2dDemacia;
import frc.robot.Field;
import frc.robot.RobotCommon;
import frc.robot.Shooter.constants.ShooterConstans;
import frc.robot.Shooter.subsystem.Shooter;

public class ShooterUtils {
  /** Creates a new shooterUtilse. */

  static Shooter shooter;

  public ShooterUtils() {

  }

  // public static Translation3d GetChassisVelAsVector() {
  // return new
  // Translation3d(RobotContainer.chassis.getRobotRelVelocities().vxMetersPerSecond,
  // RobotContainer.chassis.getRobotRelVelocities().vyMetersPerSecond, 0);
  // }

  public static double distensFromToPose2dPoint(Pose2dDemacia from, Pose2dDemacia to) {
    return from.getTranslation().getDistance(to.getTranslation());
  }

  public static double angle_betuenTowPose2d(Pose2dDemacia from, Pose2dDemacia to) {
    return to.getTranslation().minus(from.getTranslation()).getAngle().getRadians();
  }

  public static Pose2dDemacia computeFuturePosition(ChassisSpeedsDemacia speeds, Pose2dDemacia currentPose, double dtSeconds) {
    return new Pose2dDemacia(currentPose.getX() + (speeds.vxMetersPerSecond * dtSeconds),
        currentPose.getY() + (speeds.vyMetersPerSecond * dtSeconds),
        currentPose.getRotation().plus(new Rotation2dDemacia(speeds.omegaRadiansPerSecond * dtSeconds)));
    
  }

  public static Translation2dDemacia getDeliveryPoint() {
    return RobotCommon.getFutureRobotPose().getY() < Field.FieldDimensions.Y_CENTER ? ShooterConstans.DELIVERY_POINT1 : ShooterConstans.DELIVERY_POINT2;
  }

}