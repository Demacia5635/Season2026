// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.motors.TalonFXMotor;
import frc.robot.Constants.ROBOT_STATE;;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new intake. */
  private TalonFXMotor motorIntake;
  private ROBOT_STATE currState = ROBOT_STATE.IDLE;
  // private TalonSRXMotor motorRoller;
  // private TalonSRXMotor motorToShooter;

  public IntakeSubsystem() {
    motorIntake = new TalonFXMotor(intakeConstants.INTAKE_CONFIG);
    putData();
    // motorRoller = new TalonSRXMotor(intakeConstans.ROLLER_CONFIG);
    // motorToShooter = new TalonSRXMotor(intakeConstans.TO_SHOOTER_CONFIG);
  }

  // public void setDutyConveyorBelt(double pow){
  // motorRoller.setDuty(pow);
  // motorToShooter.setDuty(pow);
  // }

  public void setDutyIntake(double pow) {
    motorIntake.setDuty(pow);
  }

  public void stopIntake() {
    motorIntake.stop();
  }

  public void setCurrState(ROBOT_STATE state) {
    currState = state;
  }

  public ROBOT_STATE getCurrState() {
    return currState;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    
    builder.addStringProperty("Intake State", () -> getCurrState().name(), null);
  }

  public void putData() {
    SendableChooser<ROBOT_STATE> chooser = new SendableChooser<>();
    for (ROBOT_STATE state : ROBOT_STATE.values()) {
      chooser.addOption(state.name(), state);
    }
    chooser.onChange(STATE -> setCurrState(STATE));
    SmartDashboard.putData("Intake State Chooser", chooser);
  }

  // public void stopRoller(double pow){
  // motorRoller.stop();
  // }

  // public void stopToShooter(){
  // motorToShooter.stop();
  // }
}
