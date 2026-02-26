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
  TagPose tag;

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
    turretMotor.configMotionMagic();
    turretMotor.configPidFf(0);
    SmartDashboard.putData("reset motor position",
        new InstantCommand(() -> turretMotor.setPosition(0)).ignoringDisable(true));
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
    super.initSendable(builder);

    builder.addBooleanProperty("Min limit", this::isAtMinLimit, null);
    builder.addBooleanProperty("Max limit", this::isAtMaxLimit, null);
    builder.addBooleanProperty("Has Calibrated", this::hasCalibrated, null);
    builder.addBooleanProperty("is turret ready", () -> isReady(), null);
  }

  public Translation2d getTurretPoseOnTheRobot() {
    return TurretConstants.TURRET_POS;
  }

  public double getTurretVelocity() {
    return turretMotor.getVelocity().getValueAsDouble();
  }

  public void setPositionPID(double wantedPosition) {
    if (!hasCalibrated)
      return;
    this.wantedAngle = wantedPosition;
    double pos = MathUtil.clamp(wantedPosition, MIN_TURRET_ANGLE, MAX_TURRET_ANGLE);
    turretMotor.setPositionVoltage(pos);
  }

  public void setPositionMotion(double wantedPosition) {
    if (!hasCalibrated)
      return;
    this.wantedAngle = wantedPosition;

    double pos = MathUtil.clamp(wantedPosition, MIN_TURRET_ANGLE, MAX_TURRET_ANGLE);
    turretMotor.setMotion(pos);
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
    if (RobotCommon.currentState == RobotStates.DeliveryNotReady
        || RobotCommon.currentState == RobotStates.DeliveryWithAutoIntake
        || RobotCommon.currentState == RobotStates.DeliveryWithoutAutoIntake)
      return Math.abs(getTurretAngle() - wantedAngle) <= Math.toRadians(5);
    return Math.abs(getTurretAngle() - wantedAngle) <= Math.toRadians(2);
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
    return turretMotor.getCurrentAngle();
  }

  public void updatePositionByLimit() {
    if (isAtMinLimit())
      turretMotor.setEncoderPosition(MIN_TURRET_ANGLE);
    if (isAtMaxLimit())
      turretMotor.setEncoderPosition(MAX_TURRET_ANGLE);
  }

  public void stop() {
    turretMotor.stop();
  }
}
