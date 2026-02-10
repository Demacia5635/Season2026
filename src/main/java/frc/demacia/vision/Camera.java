// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demacia.vision;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

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
    private Translation3d turretToCamPosition;
    private boolean isCroping;
    private boolean isObjectCamera = false;
    private Translation3d robotToTurretPosition;


    public Camera(String name, Translation3d robotToCamPosition, double pitch, double yaw, boolean isCroping, boolean isObjectCamera) {
        this.name = name;
        this.robotToCamPosition = robotToCamPosition;
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
    public Camera(String name, Translation3d robotToTurretPosition, double pitch, double yaw, boolean isObjectCamera,Rotation2d turretAngle,Translation3d turretToCamPosition) {
        this.name = name;
        this.robotToTurretPosition = robotToTurretPosition;
        this.turretToCamPosition = turretToCamPosition;
        this.pitch = pitch;
        this.yaw = yaw;
        this.isOnTurret = true;
        this.turretAngle = turretAngle;
        this.tableName = "limelight-"+name;
        isCroping = false;
    }

    public Translation3d getTurretToCamPosition(){
        return turretToCamPosition;
    }
    public void setTurretAngle(Rotation2d turretAngle){
        this.turretAngle = turretAngle;
    }

    public boolean getIsOnTurret(){
        return isOnTurret;
    }

    public Rotation2d getTurrentAngle(){

        return turretAngle;
    }

    public Translation3d getRobotToCamPosition() {
        return  robotToCamPosition;
    }
    public Translation3d getRobotToTurretPosition(){
        return robotToTurretPosition;
    }

    public double getHeight() {
        return !isOnTurret ? robotToCamPosition.getZ() : robotToCamPosition.getZ()+turretToCamPosition.getZ();
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