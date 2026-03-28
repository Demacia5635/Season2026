// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.demacia.utils.log.LogManager;
import frc.demacia.utils.motors.TalonFXMotor;
import frc.robot.intake.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  private static IntakeSubsystem instance;
  public static IntakeSubsystem getInstance() {
    if (instance == null)
      instance = new IntakeSubsystem();
    return instance;
  }

  private TalonFXMotor motorIntake;

  private IntakeSubsystem() {
    motorIntake = new TalonFXMotor(IntakeConstants.INTAKE_CONFIG);
    SmartDashboard.putData("Intake", this);
    
    setName("Intake");

    LogManager.log("Intake Initalize");
  }

  public void checkElectronics() {
    motorIntake.checkElectronics();
  }

  public void setNeutralMode(boolean isBrake) {
    motorIntake.setNeutralMode(false);
  }

  public void setDutyIntake(double pow) {
    motorIntake.setDuty(pow);
  }

  public double getVelocity() {
    return motorIntake.getCurrentVelocity();
  }

  public double getCurrent(){
    return motorIntake.getCurrentCurrent();
  }
  public void stopIntake() {
    motorIntake.stop();
  }

}
