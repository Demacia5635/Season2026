// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter.subsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.log.LogManager;
import frc.demacia.utils.log.LogEntryBuilder.LogLevel;
import frc.demacia.utils.motors.TalonFXMotor;
import frc.demacia.utils.sensors.AnalogEncoder;
import frc.demacia.utils.sensors.DigitalEncoder;
import frc.demacia.utils.sensors.LimitSwitch;
import frc.robot.RobotContainer;
import frc.robot.Shooter.ShooterConstans;
import frc.robot.Shooter.ShooterConstans.ShooterState;

public class Shooter extends SubsystemBase {
  /** Creates a new shooter. */

  private TalonFXMotor shooterMotor;
  private TalonFXMotor fedderMotor;
  private TalonFXMotor hoodMotor;

  private DigitalEncoder hoodEncoder;

  private ShooterState currentShooterState = ShooterState.IDLE;
  public double angle;

  public Shooter() {
    hoodMotor = new TalonFXMotor(ShooterConstans.HOOD_CONFIG);
    shooterMotor = new TalonFXMotor(ShooterConstans.SHOOTER_MOTOR_CONFIG);
    fedderMotor = new TalonFXMotor(ShooterConstans.FEEDER_CONFIG);
    hoodEncoder = new DigitalEncoder(ShooterConstans.HOOD_ENCODER_CONFIG);
    hoodMotor.configPidFf(0);
    shooterMotor.configPidFf(0);
    SmartDashboard.putData("Shooter", this);
  }

  public boolean shooterCloseLoppCanShoote() {
    if (shooterMotor.getClosedLoopError().getValueAsDouble() < 0.3) {
      return true;
    } else {
      return false;
    }
  }

  public boolean HoodCloseLoopError() {
    if (hoodMotor.getClosedLoopError().getValueAsDouble() < Math.toRadians(0.5)) {
      return true;
    } else {
      return false;
    }
  }

  // public boolean chassisSpeedCeack(){
  // // return RobotContainer.chassis.getVelocityAsVector().getNorm() < 1.7;
  // }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("get angle", () -> Math.toDegrees(getHoodAngle()), null);
    builder.addDoubleProperty("get Vel", () -> getShooterVelocity(), null);
    builder.addBooleanProperty("Shooter close loop", () -> shooterCloseLoppCanShoote(), null);
    builder.addBooleanProperty("hoodClose loop", () -> HoodCloseLoopError(), null);
    // builder.addBooleanProperty("is it at speed", () -> chassisSpeedCeack(),
    // null);
    builder.addDoubleProperty("hood angle", () -> getHoodAngle(), null);
    builder.addBooleanProperty("is encode conected", () -> hoodEncoder.isConnected(), null);
    // LogManager.add("is encoder dedectad", null, LogLevel.LOG_AND_NT_NOT_IN_COMP,
    // getName(), hoodEncoder.isConnected());
  }

  public void setHoodMotorPosition(double position) {
    hoodMotor.setEncoderPosition(position);
  }

  public void setFlywheelVel(double speed) {
    shooterMotor.setVelocity(speed);
  }

  public double getShooterVelocity() {
    return shooterMotor.getVelocity().getValueAsDouble();
  }

  public void setFlywheelPower(double power) {
    shooterMotor.set(power);
  }

  public void setHoodPower(double power) {
    hoodMotor.set(power);
  }

  public void setHoodAngle(double angle) {
    angle = MathUtil.clamp(angle, ShooterConstans.MIN_ANGLE_HOOD, ShooterConstans.MAX_ANGLE_HOOD);
    hoodMotor.setMotion(getHoodAngleMotor() + (angle-getHoodAngle()));
    SmartDashboard.putNumber("Hood Target", angle);
  }

  public double getHoodAngleMotor(){
    return hoodMotor.getPosition().getValueAsDouble();
  }
  public double getHoodAngle() {
    return (hoodEncoder.get() * 0.5) + ShooterConstans.HOOD_OFFSET;
  }

  public void setVelocitiesAndAngle(double vel, double angle) {
    // this.angle = angle;
    setFlywheelVel(vel);
    setHoodAngle(angle);
  }

  public void setIndexerVel(double vel) {
    fedderMotor.setVelocity(vel);
  }

  public boolean canShoot() {
    // double norm = RobotContainer.chassis.getVelocityAsVector().getNorm();
    // if (norm > 0.3)

    // LogManager.log("norm: " + norm + " is hood ready: "
    // + (Math.abs(shooterMotor.getClosedLoopError().getValueAsDouble()) < 0.3)
    // + " is flywheel ready: " +
    // (Math.abs(hoodMotor.getClosedLoopError().getValueAsDouble()) < Math
    // .toRadians(0.5))
    // + " is pointing at target: " + RobotContainer.chassis.isPointingAtTarget());
    return isShooterReady();
    // norm < 2 &&
    // && RobotContainer.chassis.i
  }

  public boolean isShooterReady() {
    return Math.abs(shooterMotor.getClosedLoopError().getValueAsDouble()) < 1 &&
        Math.abs(hoodMotor.getClosedLoopError().getValueAsDouble()) < Math.toRadians(1);
  }

  public void stop() {
    shooterMotor.stopMotor();
  }

  // shooter pose on the robot
  // public Translation2d ShooterPoseOnRobot() {
  // return new Translation2d(ShooterConstans.shooterDistensFromChassis,
  // RobotContainer.chassis.getGyroAngle());
  // }

  public ShooterState getCurrentShooterState() {
    return currentShooterState;
  }

  public void setCurrentShooterCommand(ShooterState state) {
    this.currentShooterState = state;
  }

  @Override
  public void periodic() {
   
  }
}
