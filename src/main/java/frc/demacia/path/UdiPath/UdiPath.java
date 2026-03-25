package frc.demacia.path.UdiPath;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.demacia.utils.Utils;

public class UdiPath extends Command {

    public static class PathPoint {
        double x;
        double y;
        double heading;
        double velocity;
        double endVelocity;
        boolean isEnd = true;
        double endDistance = 0.1;
        PathPoint nextPoint = null;
        PathPoint prevPoint = null;
        double nextAngle = 0;

        public PathPoint(double x, double y, double heading, double endVelocity, double velocity) {
            this.x = x;
            this.y = y;
            this.heading = heading;
            this.endVelocity = endVelocity;
            this.velocity = velocity;
        }

        void setNextPoint(PathPoint nextPoint) {
            this.nextPoint = nextPoint;
            nextPoint.prevPoint = this;
            isEnd = false;
            nextAngle = Math.atan2(nextPoint.y - y, nextPoint.x - x);
            double a = Math.abs(nextAngle) > Math.PI / 2 ? Math.PI / 2 : Math.abs(nextAngle);
            endDistance = 0.5 * Math.sin(a) + 0.1;
        }
    }

    PathPoint startPoint = null;
    PathPoint endPoint = null;
    PathPoint currentPoint = null;
    Supplier<Pose2d> poseSupplier;
    Supplier<ChassisSpeeds> speedSupplier;
    Consumer<ChassisSpeeds> speedConsumer;
    double maxAccel;
    double maxAngleChange = 2*Math.PI*0.02;
    boolean atEnd = false;

    public UdiPath(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speedSupplier, Consumer<ChassisSpeeds> speedConsumer, double maxAccel, Subsystem ... requirements) {
        addRequirements(requirements);
        this.maxAccel = maxAccel;
    }

    public UdiPath addPoint(double x, double y, double heading, double endVelocity, double velocity) {
        PathPoint newPoint = new PathPoint(x, y, heading, endVelocity, velocity);
        endPoint.setNextPoint(newPoint);
        endPoint = newPoint;
        atEnd = false;
        return this;
    }

    private void setHeadings(double startHeading) {
        // set the heading if set to invalid value
        double lastHeading = startHeading;
        for(PathPoint p = startPoint; p != null; p = p.nextPoint) {
            if(p.heading > Math.PI || p.heading < -Math.PI) {
                // look for next valid heading
                double nextValidHeading = lastHeading;
                for(PathPoint pp = p.nextPoint; pp != null; pp = pp.nextPoint) {
                    if(pp.heading <= Math.PI && pp.heading >= -Math.PI) {
                        nextValidHeading = pp.heading;
                        break;
                    }
                }
                p.heading = nextValidHeading;
            }
            lastHeading = p.heading;
        }
    }

    @Override
    public void initialize() {
        currentPoint = startPoint;
        setHeadings(poseSupplier.get().getRotation().getRadians());
        atEnd = false;
    }

    private void setSpeed(double angle, double velocity, double omega) {
        speedConsumer.accept(new ChassisSpeeds(velocity * Math.cos(angle), velocity * Math.sin(angle), omega));
    }

    @Override
    public void execute() {
        Pose2d pose = poseSupplier.get();
        ChassisSpeeds speeds = speedSupplier.get();
        double currentVelocity = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        double currentAngle = Math.atan2(speeds.vyMetersPerSecond, speeds.vxMetersPerSecond);
        double distanceToPoint = Math.hypot(currentPoint.x - pose.getX(), currentPoint.y - pose.getY());
        if(distanceToPoint < currentPoint.endDistance) {
            if(currentPoint.isEnd) {
                setSpeed(currentAngle, currentPoint.endVelocity, (currentPoint.heading - pose.getRotation().getRadians()) * 2.0);
                atEnd = true;
                return;
            } else {
                currentPoint = currentPoint.nextPoint;
                distanceToPoint = Math.hypot(currentPoint.x - pose.getX(), currentPoint.y - pose.getY());
            }
        }
        double angleToPoint = Math.atan2(currentPoint.y - pose.getY(), currentPoint.x - pose.getX());
        double vel = Utils.nextVelocity(currentVelocity,currentPoint.endVelocity, currentPoint.velocity, maxAccel, distanceToPoint);
        double angle = MathUtil.clamp(angleToPoint, currentAngle + maxAngleChange, currentAngle - maxAngleChange);
        setSpeed(angle, vel, (currentPoint.heading - pose.getRotation().getRadians()) * 2.0);
    }
    @Override
    public boolean isFinished() {
        return atEnd;
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    public static UdiPath example() {
        return new UdiPath(null, null, null, 6  )
            .addPoint(1, 0, 0, 2, 4)
            .addPoint(1, 1, Math.PI/2, 2, 2)
            .addPoint(0, 1, Math.PI, 3,  1);        
    }

}
