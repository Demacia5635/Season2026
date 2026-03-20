// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.chassis.Chassis;
import frc.demacia.utils.log.LogManager;
import frc.demacia.utils.motors.TalonFXMotor;
import frc.demacia.utils.sensors.LimitSwitch;
import frc.demacia.vision.TagPose;
import frc.robot.RobotCommon;
import frc.robot.RobotCommon.RobotStates;

import static frc.robot.Turret.TurretConstants.*;

public class Turret extends SubsystemBase {
  private static Turret instance;

  public static Turret getInstance() {
    if (instance == null)
      instance = new Turret();
    return instance;
  }

  private TalonFXMotor turretMotor;

  private LimitSwitch limitSwitchMin;
  private LimitSwitch limitSwitchMax;

  private boolean isTurretLock = false;

  private double wantedAngle = 0;

  private boolean hasCalibrated = false;

  private Turret() {
    turretMotor = new TalonFXMotor(TURRET_MOTOR_CONFIG);
    limitSwitchMin = new LimitSwitch(LIMIT_SWITCH_MIN_CONFIG);
    limitSwitchMax = new LimitSwitch(LIMIT_SWITCH_MAX_CONFIG);

    SmartDashboard.putData("Turret", this);
    SmartDashboard.putData("Turret/Motor/set coast",
        new InstantCommand(() -> setNeutralMode(false)).ignoringDisable(true));
    SmartDashboard.putData("Turret/Motor/set brake",
        new InstantCommand(() -> setNeutralMode(true)).ignoringDisable(true));

    LogManager.log("Turret Initalize");
  }

  public void checkElectronics() {
    turretMotor.checkElectronics();
    limitSwitchMax.checkElectronics();
    limitSwitchMin.checkElectronics();
  }

  public void setNeutralMode(boolean isBrake) {
    turretMotor.setNeutralMode(isBrake);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addBooleanProperty("turret lock", this::getTurretLock, this::setTurretLock);
    builder.addBooleanProperty("Min limit", this::isAtMinLimit, null);
    builder.addBooleanProperty("Max limit", this::isAtMaxLimit, null);
    builder.addBooleanProperty("Has Calibrated", this::hasCalibrated, null);
    builder.addBooleanProperty("is ready", this::isReady, null);
    builder.addDoubleProperty("Position", () -> Math.toDegrees(getTurretAngle()), null);
    builder.addDoubleProperty("error", () -> Math.toDegrees(wantedAngle - getTurretAngle()), null);
  }

  public double getTurretVelocity() {
    return turretMotor.getCurrentVelocity();
  }

  public boolean getTurretLock() {
    return isTurretLock;
  }

  public void setTurretLock(boolean lock) {
    this.isTurretLock = lock;
  }

  private double moduloAngleToTurret(double angle) {
    return MathUtil.inputModulus(angle, 0, Math.PI * 2);
  }

  private double clampAngle(double angle) {
    return MathUtil.clamp(angle, TurretConstants.MIN_TURRET_ANGLE, TurretConstants.MAX_TURRET_ANGLE);
  }


  public void setPositionFieldRelative(double fieldRelativeAngle){
    setPositionPID(fieldRelativeAngle - RobotCommon.futureRobotPose.getRotation().getRadians());


  }

  public void setPositionPID(double wantedPosition) {
    if (!hasCalibrated)
      return;

    if (getTurretLock()) {
      return;
    }
    wantedPosition = clampAngle(moduloAngleToTurret(wantedPosition));

    this.wantedAngle = wantedPosition;

    if (Math.abs(wantedPosition - getTurretAngle()) < MAX_ALLOWED_ANGLE_ERROR) {
      turretMotor.stop();
      return;
    }
    turretMotor.setPositionVoltage(wantedPosition);
  }

  public void setPositionMotion(double wantedPosition) {
    if (!hasCalibrated)
      return;

    if (getTurretLock())
      return;

    this.wantedAngle = wantedPosition;
    wantedPosition = clampAngle(moduloAngleToTurret(wantedPosition));
    // if (Math.abs(wantedPosition - getTurretAngle()) <= MAX_ALLOWED_ANGLE_ERROR) {
    // turretMotor.stop();
    // return;
    // }

    turretMotor.setMotion(wantedPosition);
  }

  public double getTurretPose() {
    return turretMotor.getCurrentPosition();
  }

  public void setPower(double power) {
    // if (!hasCalibrated) return;
    // if(getTurretPose() > Math.toRadians(110) || getTurretPose() <
    // -Math.toRadians(110)) {
    // turretMotor.stop();
    // return;
    // }
    turretMotor.set(power);
  }

  public boolean isReady() {
    return hasCalibrated() && Math.abs(getTurretAngle() - wantedAngle) <= Math.toRadians(5);
    // return hasCalibrated() && (RobotCommon.currentState == RobotStates.Delivery)
    // ? Math.abs(getTurretAngle() - wantedAngle) <= Math.toRadians(5)
    // : Math.abs(getTurretAngle() - wantedAngle) <= Math.toRadians(5);
  }

  @Override
  public void periodic() {
    // if (turretMotor.getCurrentCurrent() > 35)
    // setTurretLock(true);
  }

  public boolean isAtMinLimit() {
    return !limitSwitchMin.get();
  }

  public boolean isAtMaxLimit() {
    // return false;
    return !limitSwitchMax.get();
  }

  public boolean hasCalibrated() {
    return this.hasCalibrated;
  }

  public void setEncoderPosition(double position) {
    turretMotor.setEncoderPosition(position);

  }

  public void setCalibrated() {
    this.hasCalibrated = true;
  }


  
  public double getTurretAngle() {
    return moduloAngleToTurret(turretMotor.getCurrentPosition());
  }

  public void updatePositionByLimit() {
    if (isAtMinLimit())
      turretMotor.setEncoderPosition(MIN_SENSOR);
    if (isAtMaxLimit())
      turretMotor.setEncoderPosition(MAX_TURRET_ANGLE);
  }

  public void stop() {
    turretMotor.stop();
  }
}
