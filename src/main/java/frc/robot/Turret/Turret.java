// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.demacia.utils.log.LogManager;
import frc.demacia.utils.motors.TalonFXMotor;
import frc.demacia.utils.sensors.LimitSwitch;
import frc.robot.RobotCommon;
import frc.robot.RobotCommon.RobotStates;
import frc.robot.Turret.TurretCommands.TurretCalibration;

public class Turret extends SubsystemBase {
  private static Turret instance;

  public static Turret getInstance() {
    if (instance == null)
      instance = new Turret();
    return instance;
  }

  private final TalonFXMotor turretMotor;

  private final LimitSwitch limitSwitchMin;
  private final LimitSwitch limitSwitchMax;

  private double wantedAngle;

  private boolean isTurretLock;

  private boolean hasCalibrated;

  private Turret() {
    turretMotor = new TalonFXMotor(TurretConstants.TURRET_MOTOR_CONFIG);
    limitSwitchMin = new LimitSwitch(TurretConstants.LIMIT_SWITCH_MIN_CONFIG);
    limitSwitchMax = new LimitSwitch(TurretConstants.LIMIT_SWITCH_MAX_CONFIG);

    wantedAngle = 0;

    isTurretLock = false;
    hasCalibrated = false;

    SmartDashboard.putData("Turret", this);
    SmartDashboard.putData("Turret/Motor/set coast",
        new InstantCommand(() -> setNeutralMode(false)).ignoringDisable(true));
    SmartDashboard.putData("Turret/Motor/set brake",
        new InstantCommand(() -> setNeutralMode(true)).ignoringDisable(true));
    SmartDashboard.putData("Turret/turret calibration", new TurretCalibration(this));

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
    builder.addDoubleProperty("Wanted", () -> Math.toDegrees(wantedAngle), null);
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

  public void setPositionPID(double wantedPosition) {
    if (!hasCalibrated)
      return;

    if (getTurretLock()) {
      return;
    }
    wantedPosition = clampAngle(moduloAngleToTurret(wantedPosition));

    this.wantedAngle = wantedPosition;

    if (Math.abs(wantedPosition - getTurretAngle()) < TurretConstants.MAX_ALLOWED_ANGLE_ERROR) {
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

    turretMotor.setMotion(wantedPosition);
  }

  public void setPower(double power) {
    turretMotor.set(power);
  }

  private int readyCounter = 10;
  private boolean isReady = false;

  public boolean isReady() {
    return isReady;
  }

  @Override
  public void periodic() {
    readyCounter ++;
    if (hasCalibrated()
        && Math.abs(wantedAngle - getTurretAngle()) <= getMaxAngleError(RobotCommon.getCurrentDistanceFromTarget())) {
          if(!isReady && readyCounter > 10) {
              isReady = true;
              readyCounter = 0;
          }
  } else { 
        if(isReady && readyCounter > 10) {
          readyCounter = 0;
          isReady = false;
        }
  }
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

  }

  public void setCalibrated() {
    this.hasCalibrated = true;
  }

  public double getTurretAngle() {
    return moduloAngleToTurret(turretMotor.getCurrentPosition());
  }

  public void updatePositionByLimit() {
    if (isAtMinLimit())
      turretMotor.setEncoderPosition(TurretConstants.MIN_SENSOR);
    if (isAtMaxLimit())
      turretMotor.setEncoderPosition(TurretConstants.MAX_SENSOR);

    turretMotor.configSoftwareLimit(TurretConstants.MIN_TURRET_ANGLE, TurretConstants.MAX_TURRET_ANGLE);
    hasCalibrated = true;
  }

  public void stop() {
    turretMotor.stop();
  }

  private double moduloAngleToTurret(double angle) {
    return MathUtil.inputModulus(angle, 0, Math.PI * 2);
  }

  private double clampAngle(double angle) {
    return MathUtil.clamp(angle, TurretConstants.MIN_TURRET_ANGLE, TurretConstants.MAX_TURRET_ANGLE);
  }

  private double getMaxAngleError(double distanceFromHub) {
    if (RobotCommon.getState().equals(RobotStates.Delivery))
      return Math.toRadians(12);
    return getMaxAngleErrorByDistance(distanceFromHub);
  }

  private double getMaxAngleErrorByDistance(double distanceFromHub) {
    return Math.toRadians(MathUtil.clamp(-distanceFromHub + 9, 3, 8));
  }
}
