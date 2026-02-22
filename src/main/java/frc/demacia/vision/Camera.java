// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demacia.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.demacia.utils.log.LogEntryBuilder.LogLevel;
import frc.robot.RobotCommon;
import frc.robot.Turret.Turret;
import frc.demacia.utils.log.LogManager;
import frc.demacia.vision.utils.TurretCameraConfig;
import frc.demacia.vision.utils.VisionConstants;

import static frc.demacia.vision.utils.VisionConstants.*;

/** Add your docs here. */
public class Camera {
  // NetworkTables communication for each camera
  private NetworkTable Table;
  private NetworkTableEntry cropEntry;
  private NetworkTableEntry pipeEntry;
  private double wantedPip = 0;
  private Field2d field;

  private double cameraHeight;
  private Translation2d robotToCenter;

  private CameraConfig config;

  public Camera(CameraConfig cameraConfig) {
    this.config = cameraConfig;
    this.cameraHeight = cameraConfig.getHeight();
    this.robotToCenter = cameraConfig.getRobotToCamPosition().toTranslation2d();

    wantedPip = 0;
    Table = NetworkTableInstance.getDefault().getTable(cameraConfig.getTableName());
    field = new Field2d();
    LogManager.addEntry("dist", this::getDistFromCamera).withLogLevel(LogLevel.LOG_AND_NT_NOT_IN_COMP).build();
    SmartDashboard.putData("field-tag" + cameraConfig.getName(), field);
  }

  protected double getCamToTagPitch() {
    return Table.getEntry("ty").getDouble(0.0);
  }

  protected double getCamToTagYaw() {
    return Math.toRadians(-Table.getEntry("tx").getDouble(0.0));
  }

  protected int getTagID() {
    return (int) Table.getEntry("tid").getInteger(0L);
  }

  protected NetworkTableEntry getCropEntry() {
    return Table.getEntry("crop");
  }

  protected NetworkTableEntry getPipelineEntry() {
    return Table.getEntry("pipeline");
  }

  public Pose2d getRobotPose2d() {
    if (config.isCroping()) {
      crop();
    }

    Pose2d pose = new Pose2d(getOriginToRobot(), RobotCommon.robotAngle);
    field.setRobotPose(pose);

    wantedPip = getDistFromCamera() > 1 ? 0 : 0;

    if (wantedPip != Table.getEntry("getpipe").getDouble(0.0)) {
      pipeEntry.setDouble(wantedPip);
    }

    return pose;
  }

  /**
   * Calculates robot position relative to field origin
   * Uses known AprilTag position and measured vector to tag
   * * @return Translation2d representing robot position on field
   */
  public Translation2d getOriginToRobot() {

    Translation2d originToTag = getTagTranslation2d();
    return originToTag.minus(getRobotToTagFieldRel());

  }
  public Translation2d getTagTranslation2d(){
    return VisionConstants.Tags.APRIL_TAG_LAYOUT.getTranslation(getTagID());
  }

  /**
   * Calculates vector from robot center to detected AprilTag
   * Accounts for camera offset from robot center
   * * @return Translation2d representing vector to tag
   */
  public Translation2d getRobotToTagFieldRel() {

    Translation2d cameraToTag = new Translation2d(getDistFromCamera(),
        new Rotation2d(getCamToTagYaw() + config.getYawOffset()));
    return (config.getRobotToCamPosition().toTranslation2d().plus(cameraToTag)).rotateBy(RobotCommon.robotAngle);

  }

  protected double getHeightDiff(int tagID){
    return  Math.abs(VisionConstants.Tags.APRIL_TAG_LAYOUT.getHeightDiff(tagID, config.getHeight()));
  }
  public double getDistFromCamera() {

    // double alpha = Math.abs(getCamToTagPitch() + config.getPitchOffset());
    // return getHeightDiff(getTagID()) / (Math.tan(alpha));
    double pitch = Math.toRadians(getCamToTagPitch() + config.getPitchOffset());
    double yaw = Math.toRadians(getCamToTagYaw());

    double trueElevation = Math.atan(Math.tan(pitch) / Math.cos(yaw));

    return getHeightDiff(getTagID()) / Math.tan(trueElevation);
  }

  private void crop() {
    double YawCrop = getYawCrop();
    double PitchCrop = getPitchCrop();
    double[] crop = { YawCrop - getCropOffset(), YawCrop + getCropOffset(), PitchCrop - getCropOffset(),
        PitchCrop + getCropOffset() };
    cropEntry.setDoubleArray(crop);
  }

  private double getCropOffset() {
    double crop = getDistFromCamera() * CROP_CONSTAT;
    return MathUtil.clamp(crop, MIN_CROP, MAX_CROP);
  }

  private double getYawCrop() {
    double TagYaw = ((-getCamToTagYaw()) + config.getYawOffset()) / 31.25;
    return TagYaw + RobotCommon.fieldRelativeSpeeds.vyMetersPerSecond * PREDICT_Y
        + RobotCommon.fieldRelativeSpeeds.omegaRadiansPerSecond * PREDICT_OMEGA;
  }

  private double getPitchCrop() {
    double TagPitch = (getCamToTagPitch()+config.getPitchOffset()) / 24.45;
    return TagPitch + RobotCommon.fieldRelativeSpeeds.vxMetersPerSecond * PREDICT_X;
  }

  private void cropStop() {
    double[] crop = { -1, 1, -1, 1 };
    cropEntry.setDoubleArray(crop);
  }

  public double getConfidence() {
    // Get the current distance to tag
    double currentDist = getDistFromCamera();

    // If we're within reliable range, give high confidence
    if (currentDist <= BEST_RELIABLE_DISTANCE) {
      return 1.0;
    }

    // Calculate how far we are into the falloff range (0 to 1)
    double normalizedDist = (currentDist - BEST_RELIABLE_DISTANCE)
        / ((WORST_RELIABLE_DISTANCE) - BEST_RELIABLE_DISTANCE);

    // Apply cubic falloff function
    return Math.pow(1 - normalizedDist, 3);
  }


  public int getTagId() {
    return (int) Table.getEntry("tid").getDouble(0.0);
  }


  public boolean isSeeTag() {
    return Table.getEntry("tv").getDouble(0.0) >= 0.1;
  }


}
