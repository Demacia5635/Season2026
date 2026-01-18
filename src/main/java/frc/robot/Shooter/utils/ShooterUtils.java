
package frc.robot.Shooter.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.chassis.Chassis;
import frc.robot.Shooter.subsystem.Shooter;

public class ShooterUtils extends SubsystemBase {
/** Creates a new shooterUtilse. */

    static Shooter shooter;
    static Chassis chassis; 

    public ShooterUtils(Chassis chassis) {
      this.chassis = chassis;
    }

    public static Translation3d GetChassisVelAsVector(){
      return new Translation3d(chassis.getRobotRelVelocities().vxMetersPerSecond, chassis.getRobotRelVelocities().vyMetersPerSecond, 0);
    }

    public static double distensFromToPose2dPoint(Pose2d from, Pose2d to){
        return from.getTranslation().getDistance(to.getTranslation());
    }

    public static double angle_betuenTowPose2d(Pose2d from, Pose2d to){
        return to.getTranslation().minus(from.getTranslation()).getAngle().getRadians();
    }

@Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
} 