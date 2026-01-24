  // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.climb.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.motors.TalonFXMotor;
import frc.robot.climb.constants.ClimbConstants;
import edu.wpi.first.util.sendable.SendableBuilder;

public class Climb extends SubsystemBase {
  private TalonFXMotor armsMotor;
    private TalonFXMotor leverMotor;


  /** Creates a new Climb. */
  public Climb() {
    armsMotor = new TalonFXMotor(ClimbConstants.ARMS_MOTOR_CONFIG);
    leverMotor = new TalonFXMotor(ClimbConstants.LEVER_MOTOR_CONFIG);
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

  public double getAngleArms(){
    return armsMotor.getCurrentAngle();
  }

   public void stopLever(){
    leverMotor.stop();
  }
  public double getAngleLever(){
    return leverMotor.getCurrentAngle();
  }
  public void setArmsAngle(double angle){
    armsMotor.setAngle(angle);
  }
  public void setLeverAngle(double angle){
    leverMotor.setAngle(angle);
  }
  public void resetArmsEncoder(){
    armsMotor.setEncoderPosition(0);
  }
  public void resetLeverEncoder(){
    leverMotor.setEncoderPosition(0);
  }
   public double getCurrentAmpersArms() {
    return armsMotor.getCurrentCurrent();
  }

  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Arms Current", this::getCurrentAmpersArms, null);
    builder.addDoubleProperty("Arms Angle", this::getAngleArms, null);
    builder.addDoubleProperty("Lever Angle", this::getAngleLever, null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}