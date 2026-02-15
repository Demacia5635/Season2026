// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demacia.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Turret.Turret;

/** Add your docs here. */

public class Camera {

    private String name;
    private Translation3d robotToCamPosition;
    private double pitch;
    private double yaw;
    private String tableName;
    // private boolean ishigher;// is higher than a tag 
    private boolean isOnTurret;
    private Rotation2d turretAngle;
    private Translation2d turretToCamPosition;
    private boolean isCroping;
    private boolean isObjectCamera = false;
    private Translation3d robotToTurretPosition;
    private double distanceToCamera;



    public Camera(String name, Translation3d robotToCamPosition, double pitch, double yaw, boolean isCroping, boolean isObjectCamera) {
        this.name = name;
        this.robotToCamPosition = robotToCamPosition;
        this.distanceToCamera = distanceToCamera;
        this.pitch = pitch;
        this.yaw = yaw;
        this.isOnTurret = false;
        this.tableName = "limelight-"+name;
        this.isCroping = isCroping;
        this.isObjectCamera = isObjectCamera;
    }

      /**
   * Camera for Turret
   * * 
   */
    public Camera(String name, Translation3d robotToTurretPosition, double pitch, double yaw, boolean isObjectCamera,double distanceToCamera) {
        this.name = name;
        this.robotToTurretPosition = robotToTurretPosition;
        this.distanceToCamera = Math.hypot(0.122,  0.143);
        this.turretToCamPosition = new Translation2d(distanceToCamera,new Rotation2d(0.122,  0.143));
        this.pitch = pitch;
        this.yaw = yaw;
        this.isOnTurret = true;
        this.tableName = "limelight-"+name;
        isCroping = false;
    }

    public Translation2d getTurretToCamPosition(){
        return turretToCamPosition;
    }

    public boolean getIsOnTurret(){
        return isOnTurret;
    }

    public Translation3d getRobotToCamPosition() {
        return robotToCamPosition != null? robotToCamPosition  : new Translation3d();
    }
    public Translation3d getRobotToTurretPosition(){
        return robotToTurretPosition;
    }

    public double getHeight() {
        return !isOnTurret ? robotToCamPosition.getZ() : robotToTurretPosition.getZ();
    }

    public double getPitch() {
        return this.pitch;
    }

    public double getYaw() {
        return this.yaw;
    }

    public String getName() {
        return this.name;
    }

    public String getTableName() {
        return this.tableName;
    }

    public boolean getIsCroping(){
        return isCroping;
    }

    public boolean getIsObjectCamera() {
        return isObjectCamera;
    }
}