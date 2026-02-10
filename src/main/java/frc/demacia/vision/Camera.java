// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demacia.vision;

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
    private Supplier<Rotation2d> turretAngle;
    private Translation3d turretToCamPosition;
    private boolean isCroping;
    private boolean isObjectCamera = false;

    public Camera(String name, Translation3d robotToCamPosition, double pitch, double yaw, Enum<?> cameraType) {
        this.name = name;
        this.robotToCamPosition = robotToCamPosition;
        this.pitch = pitch;
        this.yaw = yaw;
        this.cameraType = cameraType;
        this.tableName = "limelight-"+name;
        this.isCroping = isCroping;
        this.isObjectCamera = isObjectCamera;
    }

      /**
   * Camera for Turret
   * * 
   */
    public Camera(String name, Translation3d robotToCamPosition,Translation3d turretToCamPosition, double pitch, double yaw,Supplier<Rotation2d> turretAngle, boolean isObjectCamera) {
        this.name = name;
        this.robotToCamPosition = robotToCamPosition;
        this.turretToCamPosition = turretToCamPosition;
        this.pitch = pitch;
        this.yaw = yaw;
        this.isOnTurret = true;
        this.turretAngle = turretAngle;
        this.tableName = "limelight-"+name;
        isCroping = false;
    }

    public Translation3d getturretToCamPosition(){
        return turretToCamPosition;
    }

    public boolean getIsOnTurret(){
        return isOnTurret;
    }

    public Supplier<Rotation2d> getTurrentAngle(){

        return turretAngle;
    }

    public Translation3d getRobotToCamPosition() {
        return !isOnTurret ? robotToCamPosition : robotToCamPosition.rotateBy(new Rotation3d(turretAngle.get().unaryMinus()));
    }

    public double getHeight() {
        return robotToCamPosition.getZ();
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
    public Enum<?> getCameraType(){return this.cameraType;}
}