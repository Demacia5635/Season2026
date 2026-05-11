package frc.demacia.path.utils;

import java.util.ArrayList;
import java.util.function.Consumer;
import java.util.function.Supplier;

import frc.demacia.path.utils.CalculateUtils.Circle;
import frc.demacia.path.utils.CalculateUtils.Line;
import frc.demacia.path.utils.SegmentNew.calcuateResult;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class ArcTrajectory {

    public static record Point (Translation2d position, double heading, double velocity, double radius, double maxVelocity, double maxAccel) {};

    ArrayList<SegmentNew> segments = new ArrayList<SegmentNew>();
    ArrayList<Point> points = new ArrayList<Point>();
    ArrayList<CalculateUtils.Circle> circles = new ArrayList<>();
    String name;
    int currentSegmentIndex = 0;    

    public ArcTrajectory(String name, Point... points) {
        this.name = name;
        for(Point p : points) {
            this.points.add(p);
        }
        circles.add(new Circle(this.points.get(0).position, 0, false));
        for(int i = 2; i < this.points.size() - 1; i++) {
            Point prev = this.points.get(i-2);
            Point current = this.points.get(i-1);
            Point next = this.points.get(i);
            Circle circle = CalculateUtils.calculateCircleCenter(prev.position, next.position, current.position, current.radius);   
            circles.add(circle);
        }
        circles.add(new Circle(this.points.get(this.points.size() - 1).position, 0, false));

        Circle c1 = circles.get(0);
        Line l = null;
        for(int i = 1; i < circles.size() - 1; i++) {
            Circle c2 = circles.get(i);
            Point p = this.points.get(i);
            Line tangent = CalculateUtils.calculateTangent(c1, c2);
            if(l != null) {
                segments.add(new ArcNew(l.to(), tangent.from(), c1.center(), c1.isLeftTurn(), p.heading, p.velocity, p.maxVelocity, p.maxAccel));
            }
            segments.add(new LegNew(tangent.from(), tangent.to(), p.heading, p.velocity, p.maxVelocity, p.maxAccel));
            l = tangent;
            c1 = c2;
        }
    }

    private ChassisSpeeds calculateChassisSpeeds(calcuateResult result, Pose2d currentPose, double heading) {
        double headingError  = heading - currentPose.getRotation().getRadians();
        double kP = 2; 
        double omega = headingError * kP;
        return new ChassisSpeeds(result.velocity() * Math.cos(result.angle()), result.velocity() * Math.sin(result.angle()), omega);
    }

    public Command getCommand(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speedSupplier, Consumer<ChassisSpeeds> speedConsumer, Subsystem ... requirements) {
        return new FunctionalCommand(
            ()->currentSegmentIndex = 0, // initialize the index
            ()->{ // execute the current segment
                if(currentSegmentIndex < segments.size()) {
                    Pose2d currentPose = poseSupplier.get();
                    SegmentNew currentSegment = segments.get(currentSegmentIndex);
                    calcuateResult result = currentSegment.calculate(currentPose, speedSupplier.get());
                    speedConsumer.accept(calculateChassisSpeeds(result, currentPose, currentSegment.getHeading()));
                    if(result.remainingDistance() < 0.1) {
                        currentSegmentIndex++;
                    }
                }
            },
            (interrupted)->{}, // end action, do nothing
            ()->currentSegmentIndex >= segments.size(), // end condition
            requirements);
    }

}
