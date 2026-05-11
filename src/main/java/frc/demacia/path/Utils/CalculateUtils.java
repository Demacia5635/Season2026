package frc.demacia.path.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class CalculateUtils {

    public static record Circle(Translation2d center, double radius, boolean isLeftTurn) {}  
    public static record Line(Translation2d from, Translation2d to) {}  
    

    public static Circle calculateCircleCenter(Translation2d from, Translation2d to, Translation2d midPoint, double radius) {
        var toFrom = from.minus(midPoint).getAngle().getRadians();
        var toTo = to.minus(midPoint).getAngle().getRadians();
        double angle = (toFrom + toTo) / 2;
        boolean isLeftTurn = MathUtil.inputModulus(toFrom + Math.PI, 0, 2*Math.PI) < MathUtil.inputModulus(toTo, 0, 2*Math.PI);
        Translation2d center = midPoint.plus(new Translation2d(radius * Math.cos(angle), radius * Math.sin(angle)));
        return new Circle(center, radius, isLeftTurn);
    }

    public static Line calculateTangent(Circle circle1, Circle circle2) {
        var vec = circle2.center.minus(circle1.center);
        double d = vec.getNorm();
        if(d == 0) {
            return null; // circles are the same
        }
        double baseAngle = vec.getAngle().getRadians();
        if(circle1.radius == 0) {
            if(circle2.radius == 0) {
                return new Line(circle1.center, circle2.center);
            } else { // tangent from point to circle
                double alpha = Math.acos(circle2.radius / d);
                double angle = baseAngle + (circle2.isLeftTurn ? alpha : -alpha);
                Translation2d tangentPoint = circle2.center.plus(new Translation2d(circle2.radius * Math.cos(angle), circle2.radius * Math.sin(angle)));
                return new Line(circle1.center, tangentPoint);
            }
        } else if(circle2.radius == 0) { // tangent from circle to point
            double alpha = Math.acos(circle1.radius / d);
            double angle = baseAngle + (circle1.isLeftTurn ? -alpha : alpha);
            Translation2d tangentPoint = circle1.center.plus(new Translation2d(circle1.radius * Math.cos(angle), circle1.radius * Math.sin(angle)));
            return new Line(tangentPoint, circle2.center);
        } else if(circle1.isLeftTurn == circle2.isLeftTurn) { // tangent between two circles in the same direction
            double delta = Math.asin((circle1.radius - circle2.radius)  / d);
            double angle = baseAngle;
            if(circle1.isLeftTurn) {
                angle += delta - Math.PI/2;
            } else {
                angle += Math.PI/2 - delta;
            }
            Translation2d tangentPoint1 = circle1.center.plus(new Translation2d(circle1.radius * Math.cos(angle), circle1.radius * Math.sin(angle)));
            Translation2d tangentPoint2 = circle2.center.plus(new Translation2d(circle2.radius * Math.cos(angle), circle2.radius * Math.sin(angle)));
            return new Line(tangentPoint1, tangentPoint2);
        } else { // tangent between two circles in different directions
            double delta = Math.asin((circle1.radius + circle2.radius)  / d);
            double angle = baseAngle;
            if(circle1.isLeftTurn) {
                angle += delta - Math.PI/2;
            } else {
                angle += Math.PI/2 - delta;
            }
            Translation2d tangentPoint1 = circle1.center.plus(new Translation2d(circle1.radius * Math.cos(angle), circle1.radius * Math.sin(angle)));
            Translation2d tangentPoint2 = circle2.center.plus(new Translation2d(-circle2.radius * Math.cos(angle), -circle2.radius * Math.sin(angle)));
            return new Line(tangentPoint1, tangentPoint2);
        }
    }

    
}
