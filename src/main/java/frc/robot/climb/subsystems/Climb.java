// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.climb.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.motors.TalonFXMotor;
import frc.demacia.utils.motors.TalonSRXMotor;
import frc.demacia.utils.sensors.DigitalEncoder;
import frc.robot.climb.constants.ClimbConstants;
import frc.robot.climb.constants.ClimbConstants.CLIMB_STATE;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climb extends SubsystemBase {
  private TalonSRXMotor armsMotor;
  private TalonFXMotor leverMotor;
  private DigitalEncoder digitalEncoder;

  private CLIMB_STATE state;

  /** Creates a new Climb. */
  public Climb() {
    armsMotor = new TalonSRXMotor(ClimbConstants.ARMS_MOTOR_CONFIG);
    leverMotor = new TalonFXMotor(ClimbConstants.LEVER_MOTOR_CONFIG);
    digitalEncoder = new DigitalEncoder(ClimbConstants.DIGITAL_ENCODER_CONFIG);
    state = CLIMB_STATE.IDLE;
     addNT();

    SmartDashboard.putData("Climb", this);
  }
  public void addNT() {
    SendableChooser<CLIMB_STATE> stateChooser = new SendableChooser<>();
    stateChooser.addOption("PREP CLIMB", CLIMB_STATE.PREP_CLIMB);
    stateChooser.addOption("CLIMB", CLIMB_STATE.CLIMB);
    stateChooser.addOption("GETOFF", CLIMB_STATE.GET_OFF_CLIMB);
    stateChooser.addOption("IDLE", CLIMB_STATE.IDLE);
    stateChooser.addOption("TESTING", CLIMB_STATE.TESTING);
    stateChooser.onChange(newState -> this.state = newState);
    SmartDashboard.putData(getName() + "Climb State Chooser", stateChooser);
    }
  

  public void setArmsDuty(double power) {
    armsMotor.setDuty(power);
  }

  public void setLeverDuty(double power) {
    leverMotor.setDuty(power);
  }

  public void stopArms() {
    armsMotor.stop();
  }

  public void stopLever() {
    leverMotor.stop();
  }

  public double getAngleLever() {
    return leverMotor.getCurrentAngle();
  }

    public double getArmsAngle() {
    return armsMotor.getCurrentAngle();
  }


  public void setArmsAngle(double angle) {
    armsMotor.setAngle(angle);
  }

  public void setLeverAngle(double angle) {
    leverMotor.setAngle(angle);
  }

  public void resetLeverEncoder() {
    leverMotor.setEncoderPosition(0);
  }

  public double getCurrentAmpersArms() {
    return armsMotor.getCurrentCurrent();
  }

  public double getCurrentAmpersLever() {
    return leverMotor.getCurrentCurrent();
  }
  public double getDigitalEncoderAngle() {
    return digitalEncoder.get();
  }

  public void setState(CLIMB_STATE state) {
    this.state = state;
  }

  public CLIMB_STATE getState() {
    return state;
  }

  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Arms Current", this::getCurrentAmpersArms, null);
    builder.addDoubleProperty("Lever Angle", this::getAngleLever, null);
    builder.addDoubleProperty("Encoder Angle", this::getDigitalEncoderAngle, null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}