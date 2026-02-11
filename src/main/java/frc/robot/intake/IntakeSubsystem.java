// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.motors.TalonFXMotor;
import frc.demacia.utils.motors.TalonSRXMotor;

public class IntakeSubsystem extends SubsystemBase {

  //the motor valio
  private TalonFXMotor motorIntake;
  private TalonSRXMotor motorIndexerClose;
  private TalonSRXMotor motorIndexerFar;
  private TalonFXMotor motorBattery;
  private TalonFXMotor motorIndexerOnTop;

  public IntakeSubsystem() {
    motorIntake = new TalonFXMotor(IntakeConstants.INTAKE_CONFIG);
    motorIndexerClose = new TalonSRXMotor(IntakeConstants.INDEXER_CLOSE_CONFIG);
    motorIndexerFar = new TalonSRXMotor(IntakeConstants.INDEXER_FAR_CONFIG);
    motorIndexerOnTop = new TalonFXMotor(IntakeConstants.INDEXER_ON_TOP_CONFIG);
    motorBattery = new TalonFXMotor(IntakeConstants.BATTERY_CONFIG);
    motorBattery.configPidFf(0);
  }

  public void setDutyIntake(double pow) {
    motorIntake.setDuty(pow);
  }

  public void stopIntake() {
    motorIntake.stop();
  }

  public void setDutyIndexerClose(double pow){
    motorIndexerClose.setDuty(pow);
  }
  public void stopIndexerClose(){
    motorIndexerClose.stop();
  } 

  public void setDutyIndexerFar(double pow){
    motorIndexerFar.setDuty(pow);
  }

  public void stopIndexerFar(){
    motorIndexerFar.stop();
  } 

  public void setDutyIndexerOnTop(double pow){
    motorIndexerOnTop.setDuty(pow);
  }

  public void stopIndexerOnTop(){
    motorIndexerOnTop.stop();
  }

  public void setEncoderPositionBattery(double position){
    motorBattery.setEncoderPosition(position);
  }

  public void setPositionBattery(double position){
    motorBattery.setPositionVoltage(position);
  }

  public void setPower(double power){
    motorBattery.set(power);
  }

  public boolean isAtMin(double angleError){
    return Math.abs(motorBattery.getCurrentPosition()) < angleError;
  }

  public boolean isAtMax(double angleError){
    return Math.abs(IntakeConstants.MAX_POSITION - motorBattery.getCurrentPosition()) < angleError;
  }
}
