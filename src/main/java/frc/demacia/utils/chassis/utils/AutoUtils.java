package frc.demacia.utils.chassis.utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.demacia.path.utils.PathPoint;
import frc.demacia.utils.chassis.Chassis;
import frc.demacia.utils.geometry.Rotation2dDemacia;
import frc.demacia.utils.geometry.Translation2dDemacia;

public class AutoUtils {

    static Chassis chassis;
    static double maxVel = 4;
    static Translation2dDemacia cornerOffsetLeft = new Translation2dDemacia(0.480, 0.93212813);
    static Translation2dDemacia cornerOffsetRight = new Translation2dDemacia(0.480, -0.93212813);
    public static final Translation2dDemacia blueReefCenter = new Translation2dDemacia(4.490, 4.035);
    public static final Translation2dDemacia redReefCenter = new Translation2dDemacia(13.058902, 4.035);
    public static final int[] reefCams = { 0, 3 };

    public AutoUtils() {
        chassis = Chassis.getInstance();
    }

    public static void addCommands(Command c, SequentialCommandGroup cmd) {
        cmd.addCommands(c);
    }

    public static PathPoint offset(Translation2dDemacia from, double x, double y, double angle, double wantedVelocity) {
        return offset(from, x, y, angle, 0, wantedVelocity);
    }

    public static PathPoint offset(Translation2dDemacia from, double x, double y, double angle, double radius, double wantedVelocity) {
        return new PathPoint(from.getX() + x, from.getY() + y, Rotation2dDemacia.fromDegrees(angle), radius, wantedVelocity);

    }

}