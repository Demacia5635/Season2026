// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.climb.subsystems;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.demacia.utils.mechanisms.BaseMechanism;
import frc.demacia.utils.motors.MotorInterface;
import frc.demacia.utils.motors.TalonFXMotor;
import frc.demacia.utils.motors.TalonSRXMotor;
import frc.demacia.utils.sensors.AnalogSensorInterface;
import frc.demacia.utils.sensors.DigitalEncoder;
import frc.demacia.utils.sensors.SensorInterface;
import frc.robot.climb.constants.ClimbConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.climb.constants.ClimbConstants.*;

public class Climb extends BaseMechanism {

  /** Creates a new Climb. */
  public Climb() {
    super(NAME, 
    new MotorInterface[] {
      new TalonSRXMotor(ClimbConstants.ARMS_MOTOR_CONFIG),
      new TalonFXMotor(ClimbConstants.LEVER_MOTOR_CONFIG)
    }, 
    new SensorInterface[] {
      new DigitalEncoder(ClimbConstants.DIGITAL_ENCODER_CONFIG)
    });
    // leverMotor.setPosition(0);
    SmartDashboard.putData("reset motor position",
        new InstantCommand(() -> getMotor(LEVER_MOTOR_NAME).setEncoderPosition(0)).ignoringDisable(true));
    SmartDashboard.putData("Climb", this);
  }

  public boolean hasCalibratedLever() {
    return getCalibration();
  }
  public void setCalibratedLever() {
    setCalibration(true);
  }
  public void setLeverPosition(double position) {
    getMotor(LEVER_MOTOR_NAME).setEncoderPosition(position);
  }

  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Arms Current", this::getCurrentAmpersArms, null);
    builder.addDoubleProperty("Lever Angle", this::getAngleLever, null);
    builder.addDoubleProperty("Encoder Angle", this::getArmEncoderAngle, null);
    builder.addDoubleProperty("", null, null);
  }

  public void leverClimb() {
    if (getAngleLever() < ClimbConstants.ANGLE_LEVER_MID) {
      setPower(LEVER_MOTOR_NAME, ClimbConstants.powerMid);
    } else if (getAngleLever() < ClimbConstants.ANGLE_LEVER_OPEN) {
      setPower(LEVER_MOTOR_NAME, ClimbConstants.powerOpen);
    } else {
      stop(LEVER_MOTOR_NAME);
    }
  }

  public void stateClose() {
    if (getArmEncoderAngle() != ClimbConstants.ARMS_ANGLE_CLOSED
        || getAngleLever() != ClimbConstants.ANGLE_LEVER_CLOSED) {
      setArmsAngle(ClimbConstants.ARMS_ANGLE_CLOSED);
      setAngle(LEVER_MOTOR_NAME, ClimbConstants.ANGLE_LEVER_CLOSED);
    }
  }

  public double getAngleLever() {
    return getMotor(LEVER_MOTOR_NAME).getCurrentAngle();
  }

  public void setArmsAngle(double angle) {
    angle = MathUtil.clamp(angle, 0, ClimbConstants.ANGLE_LEVER_CLOSED);
    double error = MathUtil.angleModulus(angle - getArmEncoderAngle());
    setVoltage(ARM_MOTOR_NAME, error * ClimbConstants.ARMS_KP);
  }

  public void setLeverAngle(double angle) {
    getMotor(LEVER_MOTOR_NAME).setPositionVoltage(angle);
  }

  public void resetLeverEncoder() {
    getMotor(LEVER_MOTOR_NAME).setEncoderPosition(0);
  }

  public double getCurrentAmpersArms() {
    return getMotor(ARM_MOTOR_NAME).getCurrentCurrent();
  }

  public double getCurrentAmpersLever() {
    return getMotor(LEVER_MOTOR_NAME).getCurrentCurrent();
  }

  public double getArmEncoderAngle() {
    return MathUtil.angleModulus(MathUtil.angleModulus(((AnalogSensorInterface) getSensor(CLIMB_ENCODER_NAME)).get()) - ClimbConstants.ARMS_OFFSET);
  }

  public Pose2d getTargetClimbPose(boolean isRed, boolean isRightClimb) {
    if (isRed) {
      return isRightClimb ? ClimbConstants.targetRightSideRed : ClimbConstants.targetLeftSideRed;
    } else {
      return isRightClimb ? ClimbConstants.targetRightSideBlue : ClimbConstants.targetLeftSideBlue;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}