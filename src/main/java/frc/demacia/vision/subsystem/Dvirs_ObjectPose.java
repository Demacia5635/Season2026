// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demacia.vision.subsystem;



import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.demacia.utils.log.LogManager;
import frc.demacia.vision.Camera;
import frc.robot.RobotCommon;

/** Add your docs here. */
public class Dvirs_ObjectPose {

    private NetworkTable Table;

    private Camera objectCam;

    private double camObjectYaw;
    private double camObjectPitch;
    private double dist;

    private Translation2d cameraToObject;
    private Translation2d robotToObject;
    private Translation2d fieldToObject;
    private Translation2d previousPose;


    public Dvirs_ObjectPose(Camera objectCam){
        this.objectCam = objectCam;
        Table = NetworkTableInstance.getDefault().getTable(objectCam.getTableName());
        SmartDashboard.putString("objectCam.getTableName()", objectCam.getTableName());
        LogManager.log("fuel_dist"+getDistance());
        LogManager.log("fuel_yaw"+getYaw());

    }

    public Translation2d giveBestTranslation(){
        if(isObjectDetected()){
            updateValues();
            getRobotToObjectFeildRel();
            getOriginToObject();
            if(previousPose != null && isPoseElegable(previousPose)){
                return previousPose;
            }
            previousPose =fieldToObject;
            return fieldToObject;
        }
        else if(previousPose != null && isPoseElegable(previousPose)){
            return previousPose;
        }

        return Translation2d.kZero;
    }

    public boolean isObjectDetected(){
        return Table.getEntry("tv").getDouble(0.0) != 0;
    }

    public void updateValues(){
        camObjectPitch = Table.getEntry("ty").getDouble(0.0);
        camObjectYaw = (-Table.getEntry("tx").getDouble(0.0));
    }

    public double getDistance(){
        double alpha = Math.abs(camObjectPitch + objectCam.getPitch());
        dist = (Math.abs(objectCam.getHeight())/ (Math.tan(Math.toRadians(alpha))));
        return dist;
    }

    public Translation2d getRobotToObjectFeildRel(){
        cameraToObject = new Translation2d(getDistance(),
            Rotation2d.fromDegrees(camObjectYaw + objectCam.getYaw()));
        robotToObject = (objectCam.getRobotToTurretPosition().toTranslation2d().plus(cameraToObject))
            .rotateBy(RobotCommon.robotAngle);
        return robotToObject;
    }
    
    public Translation2d getOriginToObject(){
        
        if (robotToObject != null) {

            fieldToObject = RobotCommon.currentRobotPose.getTranslation().minus(getRobotToObjectFeildRel());

            return fieldToObject;
        }
        return new Translation2d();
    }

    public boolean isPoseElegable(Translation2d pose){
        double dist = Math.sqrt((Math.pow(pose.getX()-RobotCommon.currentRobotPose.getX(),2)+Math.pow(pose.getY()-RobotCommon.currentRobotPose.getY(),2)));
        if(robotToObject != null){
            double distFromCurretTarget = Math.sqrt((Math.pow(pose.getX()-fieldToObject.getX(),2)+Math.pow(pose.getY()-fieldToObject.getY(),2)));
            return dist <0.2 && dist <3.3 && distFromCurretTarget> 7;
        }
        return dist <0.2 && dist <3.3;
    }
    public double getYaw(){
        return (-Table.getEntry("tx").getDouble(0.0));
    }
    public double getPitch(){
        return (Table.getEntry("ty").getDouble(0.0));
    }
}