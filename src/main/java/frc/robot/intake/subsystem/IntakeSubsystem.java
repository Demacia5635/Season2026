// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake.subsystem;


import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.motors.TalonFXMotor;
import frc.demacia.utils.motors.TalonSRXMotor;
import frc.robot.intake.intakeConstans;
import frc.robot.intake.intakeConstans.INTAKE_STATE;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new intake. */
  private TalonFXMotor motorIntake;
  private TalonSRXMotor motorRoller;
  private TalonSRXMotor motorToShooter;
  private INTAKE_STATE currState = INTAKE_STATE.IDLE;

  public IntakeSubsystem() {
    motorIntake = new TalonFXMotor(intakeConstans.INTAKE_CONFIG);
    motorRoller = new TalonSRXMotor(intakeConstans.ROLLER_CONFIG);
    motorToShooter = new TalonSRXMotor(intakeConstans.TO_SHOOTER_CONFIG);
  }

  public void setDutyConveyorBelt(double pow){
    setDutyRoller(pow);
    setDutyToShooter(pow);
  }

  public void setDutyIntake(double pow){
    motorIntake.setDuty(pow);
  }

  public void setDutyRoller(double pow){
    motorRoller.setDuty(pow);
  }

  public void setDutyToShooter(double pow){
    motorToShooter.setDuty(pow);
  }

  public void setState(INTAKE_STATE state){
    currState = state;
  }

  public INTAKE_STATE getCurrentState(){
    return currState;
  }

  public void putData() {
    SendableChooser<INTAKE_STATE> stateChooser = new SendableChooser<>();
    for(INTAKE_STATE state : INTAKE_STATE.values()) {
      stateChooser.addOption(state.name(), state);
    }
    stateChooser.onChange(STATE -> setState(STATE));
    SmartDashboard.putData("Intake State", stateChooser);
  }
}
