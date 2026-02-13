// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter.subsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.motors.TalonFXMotor;
import frc.demacia.utils.sensors.DigitalEncoder;
import frc.robot.Shooter.ShooterConstans;
import frc.robot.Shooter.ShooterConstans.ShooterState;

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

  private ShooterState currentShooterState = ShooterState.IDLE;
  public double angle;

  public Shooter() {
    hoodMotor = new TalonFXMotor(ShooterConstans.HOOD_CONFIG);
    shooterMotor = new TalonFXMotor(ShooterConstans.SHOOTER_MOTOR_CONFIG);
    feederMotor = new TalonFXMotor(ShooterConstans.FEEDER_CONFIG);
    hoodEncoder = new DigitalEncoder(ShooterConstans.HOOD_ENCODER_CONFIG);
    setHoodMotorPosition(getHoodAngle());
    hoodMotor.configPidFf(0);
    hoodMotor.configMotionMagic();
    shooterMotor.configPidFf(0);
    SmartDashboard.putData("Shooter", this);
    SmartDashboard.putData("Shooter/Hood/set coast", new InstantCommand(() -> hoodMotor.setNeutralMode(false)).ignoringDisable(true));
    SmartDashboard.putData("Shooter/Hood/set brake", new InstantCommand(() -> hoodMotor.setNeutralMode(true)).ignoringDisable(true));
    SmartDashboard.putData("Shooter/FlyWheel/set coast", new InstantCommand(() -> shooterMotor.setNeutralMode(false)).ignoringDisable(true));
    SmartDashboard.putData("Shooter/FlyWheel/set brake", new InstantCommand(() -> shooterMotor.setNeutralMode(true)).ignoringDisable(true));
    SmartDashboard.putData("Shooter/Feeder/set coast", new InstantCommand(() -> feederMotor.setNeutralMode(false)).ignoringDisable(true));
    SmartDashboard.putData("Shooter/Feeder/set brake", new InstantCommand(() -> feederMotor.setNeutralMode(true)).ignoringDisable(true));
  }

  /**
   * 
   * this funcsan is to set all the shooter  to const or brake
   */
  public void setNeutralMode(boolean isBrake) {
    feederMotor.setNeutralMode(isBrake);
    shooterMotor.setNeutralMode(isBrake);
    hoodMotor.setNeutralMode(isBrake);
  }

  /**
   * this funcsan is to get if the fly weel is at the velwe sood be in error of 0.03
   */
  public boolean shooterCloseLoppCanShoote() {
    if (shooterMotor.getClosedLoopError().getValueAsDouble() < 0.3) {
      return true;
    } else {
      return false;
    }
  }

  /**
   * this funcsan is to see if the hood isnin the angle is siid be in error of 0.5 degrey
   */
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
    builder.addDoubleProperty("abs encoder", ()->(MathUtil.angleModulus(hoodEncoder.get()) * 0.5) , null);
    // LogManager.add("is encoder dedectad", null, LogLevel.LOG_AND_NT_NOT_IN_COMP,
    // getName(), hoodEncoder.isConnected());
  }

  /**
   * this funcsean set the encoder posr
   * @param position is the pose yuo want the encoder to be
   */
  public void setHoodMotorPosition(double position) {
    hoodMotor.setEncoderPosition(position);
  }

  /**
   * this funcsan is for set the fly wheel vel
   * @param wantedSpeed is the speed you want the fly weel to be
   */
  public void setFlywheelVel(double wantedSpeed) {
    if (Double.isNaN(wantedSpeed)) return;
    double vel = MathUtil.clamp(wantedSpeed, 0, 20);
    shooterMotor.setVelocity(vel);
  }

  /**
   * this funcsan is to get the fly weel vel
   */
  public double getShooterVelocity() {
    return shooterMotor.getVelocity().getValueAsDouble();
  }

  /**
   * this funcsan is to set the fly weel power
   * @param power is to set the fly weel power
   */
  public void setFlywheelPower(double power) {
    shooterMotor.set(power);
  }

  /**
   * this funcsan is to set the hood power
   * @param power is the vaubole of the pwer of the hood
   */
  public void setHoodPower(double power) {
    hoodMotor.set(power);
  }

  /**
   * this funcsan is to set the hood angle
   * @param wantedAngle is the funcsan to set the hood angle in redeans
   */
  public void setHoodAngle(double wantedAngle) {
    if (Double.isNaN(wantedAngle)) return;

    double angle = MathUtil.clamp(wantedAngle, ShooterConstans.MIN_ANGLE_HOOD, ShooterConstans.MAX_ANGLE_HOOD);
    
    if(Math.abs(angle - getHoodAngle()) < Math.toRadians(0.5)){
      hoodMotor.stop();
      return;
    }
    hoodMotor.setMotion(angle);
  }

  /**
   * this funcsan is to get the hood motor angle
   */
  public double getHoodAngleMotor() {
    return hoodMotor.getPosition().getValueAsDouble();
  }

  /**
   * this funcsan is the get the acsole hood angle
   */
  public double getHoodAngle() {
    return MathUtil.angleModulus((MathUtil.angleModulus(hoodEncoder.get()) * 0.5) + ShooterConstans.HOOD_OFFSET);
    // return (hoodEncoder.get() * 0.5) + ShooterConstans.HOOD_OFFSET;
  }


  /**
   * this funcsan is to set the angle and vel togeder
   * @param vel this is the vel valio
   * @param angle this is the angke valiuo`
   */
  public void setVelocitiesAndAngle(double vel, double angle) {
    // this.angle = angle;
    setFlywheelVel(vel);
    setHoodAngle(angle);
  }

  /*this is set the feeder power */
  public void setFeederPower(double power) {
    feederMotor.set(power);
  }

  // public void setIndexerVel(double vel) {
  // feederMotor.setVelocity(vel);
  // }
  /**this funcsan is to see if the shooter can shoot*/
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

  /**this funcsan is to see if the shooter ready */
  public boolean isShooterReady() {
    return Math.abs(shooterMotor.getClosedLoopError().getValueAsDouble()) < 1 &&
        Math.abs(hoodMotor.getClosedLoopError().getValueAsDouble()) < Math.toRadians(1);
  }

  /**this funcsan is to stop the shooter */
  public void stop() {
    shooterMotor.stopMotor();
    hoodMotor.stopMotor();
    feederMotor.stopMotor();
  }

  // shooter pose on the robot
  // public Translation2d ShooterPoseOnRobot() {
  // return new Translation2d(ShooterConstans.shooterDistensFromChassis,
  // RobotContainer.chassis.getGyroAngle());
  // }

  /**this funcsan is to get the shooter state */
  public ShooterState getCurrentShooterState() {
    return currentShooterState;
  }

  /**this funcsan is to set the state */
  public void setCurrentShooterCommand(ShooterState state) {
    this.currentShooterState = state;
  }

  @Override
  public void periodic() {

  }
}
