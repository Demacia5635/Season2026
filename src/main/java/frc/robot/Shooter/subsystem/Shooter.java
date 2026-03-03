// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter.subsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.motors.TalonFXMotor;
import frc.demacia.utils.sensors.DigitalEncoder;
import frc.robot.RobotCommon;
import frc.robot.RobotCommon.RobotStates;
import frc.robot.Shooter.constants.ShooterConstans;

/**
 * this is the subsystem of the shooter
 * 
 * hear we crate all the motors and the sensorse and use them
 */
public class Shooter extends SubsystemBase {
  /** Creates a new shooter. */

  private TalonFXMotor shooterMotor;
  private TalonFXMotor feederMotor;
  private TalonFXMotor hoodMotor;

  private DigitalEncoder hoodEncoder;

  public double angle;

  private static Shooter instance;

  private Shooter() {
    hoodMotor = new TalonFXMotor(ShooterConstans.HOOD_CONFIG);
    shooterMotor = new TalonFXMotor(ShooterConstans.SHOOTER_MOTOR_CONFIG);
    feederMotor = new TalonFXMotor(ShooterConstans.FEEDER_CONFIG);
    hoodEncoder = new DigitalEncoder(ShooterConstans.HOOD_ENCODER_CONFIG);
    setHoodMotorPosition(getHoodAngleAbsEncoder());
    hoodMotor.configPidFf(0);
    hoodMotor.configMotionMagic();
    shooterMotor.configPidFf(0);
    SmartDashboard.putData("Shooter", this);
    SmartDashboard.putData("Shooter/Hood/set coast",
        new InstantCommand(() -> hoodMotor.setNeutralMode(false)).ignoringDisable(true));
    SmartDashboard.putData("Shooter/Hood/set brake",
        new InstantCommand(() -> hoodMotor.setNeutralMode(true)).ignoringDisable(true));
    SmartDashboard.putData("Shooter/FlyWheel/set coast",
        new InstantCommand(() -> shooterMotor.setNeutralMode(false)).ignoringDisable(true));
    SmartDashboard.putData("Shooter/FlyWheel/set brake",
        new InstantCommand(() -> shooterMotor.setNeutralMode(true)).ignoringDisable(true));
    SmartDashboard.putData("Shooter/Feeder/set coast",
        new InstantCommand(() -> feederMotor.setNeutralMode(false)).ignoringDisable(true));
    SmartDashboard.putData("Shooter/Feeder/set brake",
        new InstantCommand(() -> feederMotor.setNeutralMode(true)).ignoringDisable(true));

  
  }

  public static Shooter getInstance() {
    if (instance == null)
      instance = new Shooter();
    return instance;
  }

  public void checkElectronics() {
    shooterMotor.checkElectronics();
    hoodMotor.checkElectronics();
    feederMotor.checkElectronics();
    hoodEncoder.checkElectronics();
  }

  /**
   * 
   * this function is to set all the shooter to const or brake
   */
  public void setNeutralMode(boolean isBrake) {
    feederMotor.setNeutralMode(isBrake);
    shooterMotor.setNeutralMode(isBrake);
    hoodMotor.setNeutralMode(isBrake);
  }

  public boolean isReady() {
    if (RobotCommon.currentState == RobotStates.DeliveryNotReady
        || RobotCommon.currentState == RobotStates.DeliveryWithAutoIntake
        || RobotCommon.currentState == RobotStates.DeliveryWithoutAutoIntake)
      return Math.abs(hoodMotor.getCurrentClosedLoopError()) <= Math.toRadians(3) && Math.abs(shooterMotor.getCurrentClosedLoopError()) < 0.8;
    return Math.abs(hoodMotor.getCurrentClosedLoopError()) <= Math.toRadians(1.5) && Math.abs(shooterMotor.getCurrentClosedLoopError()) < 0.4;
  }  

  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("get angle", () -> Math.toDegrees(getHoodAngleAbsEncoder()), null);
    builder.addDoubleProperty("get Vel", () -> getShooterVelocity(), null);
    builder.addBooleanProperty("Is hood ready", ()->(Math.abs(hoodMotor.getCurrentClosedLoopError()) <= Math.toRadians(1) ),null);
    builder.addBooleanProperty("Is shooter ready", ()->(Math.abs(shooterMotor.getCurrentClosedLoopError()) < 0.5),null);
    
    // builder.addBooleanProperty("is it at speed", () -> chassisSpeedCeack(),
    // null);
    builder.addDoubleProperty("hood angle", () -> getHoodAngle(), null);
    builder.addBooleanProperty("is encode conected", () -> hoodEncoder.isConnected(), null);
    builder.addDoubleProperty("abs encoder", () -> (MathUtil.angleModulus(hoodEncoder.get()) * 0.5), null);
  }

  /**
   * this funcsean set the encoder posr
   * 
   * @param position is the pose yuo want the encoder to be
   */
  public void setHoodMotorPosition(double position) {
    hoodMotor.setEncoderPosition(position);
  }

  /**
   * this function is for set the fly wheel vel
   * 
   * @param wantedSpeed is the speed you want the fly weel to be
   */
  public void setFlywheelVel(double wantedSpeed) {
    if (Double.isNaN(wantedSpeed))
      return;
    double vel = MathUtil.clamp(wantedSpeed, 0, 20);
    shooterMotor.setVelocity(vel);
  }

  /**
   * this function is to get the fly weel vel
   */
  public double getShooterVelocity() {
    return shooterMotor.getVelocity().getValueAsDouble();
  }

  /**
   * this function is to set the fly weel power
   * 
   * @param power is to set the fly weel power
   */
  public void setFlywheelPower(double power) {
    shooterMotor.set(power);
  }

  /**
   * this function is to set the hood power
   * 
   * @param power is the vaubole of the pwer of the hood
   */
  public void setHoodPower(double power) {
    hoodMotor.set(power);
  }

  /**
   * this function is to set the hood angle
   * 
   * @param wantedAngle is the function to set the hood angle in redeans
   */
  public void setHoodAngle(double wantedAngle) {
    if (Double.isNaN(wantedAngle))
      return;

    double angle = MathUtil.clamp(wantedAngle, ShooterConstans.MIN_ANGLE_HOOD, ShooterConstans.MAX_ANGLE_HOOD);

    if (Math.abs(angle - getHoodAngleAbsEncoder()) < ShooterConstans.MAX_HOOD_ANGLE_ERROR) {
      hoodMotor.stop();
      return;
    }
    hoodMotor.setPositionVoltage(angle);
  }

  /**
   * this function is to get the hood motor angle
   */
  public double getHoodAngle() {
    return hoodMotor.getPosition().getValueAsDouble();
  }

  /**
   * this function is the get the acsole hood angle
   */
  public double getHoodAngleAbsEncoder() {
    return MathUtil.angleModulus((MathUtil.angleModulus(hoodEncoder.get()) * 0.5) + ShooterConstans.HOOD_OFFSET);
  }

  /**
   * this function is to set the fly wheel vel and the hood angle at the same time
   * 
   * @param vel   wanted velocity in m/s
   * @param angle wanted hood angle in radians
   */
  public void setVelocitiesAndAngle(double vel, double angle) {
    setFlywheelVel(vel);
    setHoodAngle(angle);
  }

  /* this is set the feeder power */
  public void setFeederPower(double power) {
    feederMotor.set(power);
  }

  /**this funcsan is to stop the shooter */
  public void stop() {
    shooterMotor.stopMotor();
    hoodMotor.stopMotor();
    feederMotor.stopMotor();
  }

  @Override
  public void periodic() {

  }
}
