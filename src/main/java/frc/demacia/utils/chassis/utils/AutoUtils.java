package frc.demacia.utils.chassis.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.demacia.path.utils.PathPoint;
import frc.demacia.path.utils.Segment;
import frc.demacia.utils.chassis.Chassis;
import frc.robot.RobotContainer;

public class AutoUtils {

    static Chassis chassis;
    static double maxVel = 4;
    static Translation2d cornerOffsetLeft = new Translation2d(0.480, 0.93212813);
    static Translation2d cornerOffsetRight = new Translation2d(0.480, -0.93212813);
    public static final Translation2d blueReefCenter = new Translation2d(4.490, 4.035);
    public static final Translation2d redReefCenter = new Translation2d(13.058902, 4.035);
    public static final int[] reefCams = { 0, 3 };

    public AutoUtils() {
        chassis = RobotContainer.chassis;
    }

    public static void addCommands(Command c, SequentialCommandGroup cmd) {
        cmd.addCommands(c);
    }

    public static PathPoint offset(Translation2d from, double x, double y, double angle) {
        return offset(from, x, y, angle, 0);
    }

    public static PathPoint offset(Translation2d from, double x, double y, double angle, double radius) {
        return new PathPoint(from.getX() + x, from.getY() + y, Rotation2d.fromDegrees(angle), radius);

    }

}