
package frc.robot.Shooter.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.geometry.Rotation2d;
import frc.demacia.utils.geometry.Translation2d;
import frc.robot.Shooter.commands.ShooterFollowCommand;
import frc.robot.Shooter.subsystem.Shooter;

public class ShooterUtils extends SubsystemBase {
/** Creates a new shooterUtilse. */

    Shooter shooter;


    public ShooterUtils() {}

    public static double distensFromToPose2dPoint(Pose2d from, Pose3d to){
        return 0;
    }

    public static double angle_betuenTowPose2d(Pose2d from, Pose3d to){
        return 0;
    }

@Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
} 