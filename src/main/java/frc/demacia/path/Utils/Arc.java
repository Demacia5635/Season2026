// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demacia.path.utils;

import frc.demacia.utils.geometry.Rotation2dDemacia;
import frc.demacia.utils.geometry.Translation2dDemacia;

/** Add your docs here. */
public class Arc extends Segment{

    //p1 represents the start point, p2 represents the circle center
    Rotation2dDemacia angle;


    final Translation2dDemacia startVector;
    final double radius;
    /**
     * 
     * @param p1 - Start point of arc
     * @param p2 - Circle center of arc
     * @param angle - Arc's angle
     */
    public Arc(Translation2dDemacia p1, Translation2dDemacia p2, Rotation2dDemacia angle, double headingInRad, double wantedVelocity)
    {
        //start point
        super(p1,p2, headingInRad, wantedVelocity);
        this.angle = angle;

        startVector = p1.minus(p2);
        radius = startVector.getNorm();
        
    }

    @Override
    public Translation2dDemacia[] getPoints()
    {
      Translation2dDemacia arrow = p1.minus(p2);
      Translation2dDemacia[] points = new Translation2dDemacia[4];
      double diffAngle = angle.getRadians();
      int place = 0;
      if(radius == 0){
        return new Translation2dDemacia[] {p1, p2};
      }
        for (double i = 0;Math.abs(i) < Math.abs(diffAngle); i = i + (diffAngle / 4)) {
            points[place] = p2.plus(arrow.rotateBy(new Rotation2dDemacia(i)));
          
            place++;
        }

        return points;
    }

    @Override
    public double getDistanceLeft(Translation2dDemacia currentPosition) {
      Translation2dDemacia relativePos = currentPosition.minus(p2);

      Rotation2dDemacia diffAngle = startVector.getAngle().minus(relativePos.getAngle());

      return Math.abs((angle.getRadians()-diffAngle.getRadians()) * radius);    
    }

    @Override
    public Translation2dDemacia calcVector(Translation2dDemacia pos,double velocity)
    {
        Translation2dDemacia relativePos = pos.minus(p2);
        double dFromCenter = relativePos.getNorm();

        Rotation2dDemacia tAngle = new Rotation2dDemacia(((velocity * 0.02) / radius) * Math.signum(angle.getDegrees()));


        //tangent angle to arc, determined by the robot's position
        Rotation2dDemacia tanAngle = relativePos.getAngle().plus(new Rotation2dDemacia(Math.toRadians(90 * Math.signum(angle.getDegrees()))));
        //fix angle = turn angle, multiplied by a ratio.
        //bigger ratio - will turn more towards the center
        //smaller ratio - will turn less towards the center
        Rotation2dDemacia fixAngle = tAngle.times(dFromCenter / radius);



      
      return new Translation2dDemacia(velocity, tanAngle.plus(fixAngle));
    }

    @Override
    public double distancePassed(Translation2dDemacia pos)
    {
      Translation2dDemacia relativePos = pos.minus(p2);

      Rotation2dDemacia diffAngle = startVector.getAngle().minus(relativePos.getAngle());

      return Math.abs(diffAngle.getRadians() * radius);
    }

   // public 


    @Override
    public double getLength()
    {
      return Math.abs(angle.getRadians()) * radius;
    }

    @Override
    public String toString() {
        return "\n~Arc~\nStartPoint : " + p1 + "\nCircleCenter : " + p2 + "\nAngle : " + angle + "\nRadius : " + radius;
    }

}