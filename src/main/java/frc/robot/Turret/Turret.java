// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.motors.TalonFXMotor;
import frc.demacia.utils.sensors.LimitSwitch;
import frc.demacia.vision.TagPose;
import frc.robot.Field;
import frc.robot.Field.Red;

import static frc.robot.Turret.TurretConstants.*;

public class Turret extends SubsystemBase {
  private TalonFXMotor turretMotor;
  private LimitSwitch limitSwitchMin;
  private LimitSwitch limitSwitchMax;

  TagPose tag;
  Field field;
  Red redSide;


  private boolean hasCalibrated = false;

  private static Turret instance;

  private Turret() {
    turretMotor = new TalonFXMotor(TURRET_MOTOR_CONFIG);
    limitSwitchMin = new LimitSwitch(LIMIT_SWITCH_MIN_CONFIG);
    limitSwitchMax = new LimitSwitch(LIMIT_SWITCH_MAX_CONFIG);
    SmartDashboard.putData("Turret",this);
    
  }
  @Override
  public void initSendable(SendableBuilder builder) {
      builder.addBooleanProperty("/Min limit", ()->isAtMinLimit(), null);
      builder.addBooleanProperty("/Max limit", ()->isAtMaxLimit(), null);
  }

  public static Turret getInstance() {
    if (instance == null)
      instance = new Turret();
    return instance;
  }

  public Translation2d getTurretPoseOnTheRobot(){
    return new Translation2d();
  }

  public double getTurretToHubAngle(){
    Translation2d cameraToTag = tag.getCameraToTag(); 
    Pose2d TagPose = new Pose2d();
    Translation2d HubPose = redSide.HUB_CENTER;
    Translation2d TagToHub = TagPose.getTranslation().minus(HubPose);
    Translation2d cameraToHub = cameraToTag.plus(TagToHub);
    Translation2d TurretToHUb = cameraToHub.minus(getTurretPoseOnTheRobot());
    return TurretToHUb.getAngle().getRadians();
  }

  public void setPositionPID(double position) {
    MathUtil.clamp(position, MIN_TURRET_ANGLE, MAX_TURRET_ANGLE);
    updatePositionByLimit();

    turretMotor.setPosition(position);
  }

  public void setPositionMotion(double position) {
    position = MathUtil.clamp(position, MIN_TURRET_ANGLE, MAX_TURRET_ANGLE);
    updatePositionByLimit();
    turretMotor.setMotion(position);
  }

  public void setPower(double power) {
    turretMotor.set(power);
  }

  public boolean isAtMinLimit() {
    return !limitSwitchMin.get();
  }

  public boolean isAtMaxLimit() {
    return !limitSwitchMax.get();
  }

  public boolean hasCalibrated() {
    return this.hasCalibrated;
  }

  public void setEncoderPosition(double position) {
    turretMotor.setEncoderPosition(position);
    setCalibrated();
  }
  public void setCalibrated() {
    this.hasCalibrated = true;
  }

  public void updatePositionByLimit() {
    if (isAtMinLimit())
      turretMotor.setEncoderPosition(MIN_TURRET_ANGLE);
    if (isAtMaxLimit())
      turretMotor.setEncoderPosition(MAX_TURRET_ANGLE);
  }
}
