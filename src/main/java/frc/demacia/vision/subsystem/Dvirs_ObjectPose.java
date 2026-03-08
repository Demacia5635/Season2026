// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demacia.vision.subsystem;



import java.time.Period;
import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.log.LogManager;
import frc.demacia.vision.Camera;
import frc.robot.RobotCommon;

/** Add your docs here. */
public class Dvirs_ObjectPose extends SubsystemBase {

    private NetworkTable Table;

    private Camera objectCam;

    private double camObjectYaw;
    private double camObjectPitch;
    private double dist;

    private Translation2d robotToCam;
    private Translation2d[] previousPos = new Translation2d[3];
    private boolean tv = false;
    private Field2d fuleField = new Field2d();

    private InterpolatingDoubleTreeMap lut = new InterpolatingDoubleTreeMap();


    public Dvirs_ObjectPose(Camera objectCam){
        super();
        lut.put(-8.0,1.30);
        lut.put(-44.0,0.22);
        lut.put(-26.0, 0.47);
        lut.put(-16.67, 0.78);
        lut.put(-12.0, 1.1);
        lut.put(-30.0, 0.4);
        lut.put(-25.0, 0.5);
        lut.put(-20.0, 0.6);
        // lut.put(-46.0 , 0.22);
        // lut.put(-37.0 , 0.33);
        // lut.put(-30.0 , 0.47);
        // lut.put(-21.5 , 0.7);
        // lut.put(-15.0 , 1.05);
        // lut.put(-11.75 , 1.32);

        Arrays.fill(previousPos, null);
        this.objectCam = objectCam;
          robotToCam = objectCam.getRobotToCamPosition().toTranslation2d();
      
        Table = NetworkTableInstance.getDefault().getTable(objectCam.getTableName());
        SmartDashboard.putString("objectCam.getTableName()", objectCam.getTableName());
        SmartDashboard.putData("Fuel", fuleField);
       
    }

    public Translation2d giveBestTranslation(){
        if(isObjectDetected()){
            updateValues();
            return getOriginToObject();
        }
        return Translation2d.kZero;
    }

    public boolean isObjectDetected(){
        return tv;
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

    public boolean checkIfInValeble(Translation2d translation2d){
        double x = translation2d.getX();
        double y = translation2d.getY();
        
        return RobotCommon.currentRobotPose.getX()+RobotCommon.currentRobotPose.getRotation().getCos()*800 < x;
    }
}
