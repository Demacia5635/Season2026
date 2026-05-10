// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.intake.subsystems.IntakeSubsystem;
import frc.robot.intake.subsystems.ShinuaSubsystem;

public class GetBallOutCommand extends Command {

  IntakeSubsystem intake;
  Trigger triggerToDisable;  
  ShinuaSubsystem shinua;

  public GetBallOutCommand(IntakeSubsystem intake, ShinuaSubsystem shinua, Trigger triggerToDisable) {
    this.intake = intake;
    this.triggerToDisable = triggerToDisable;
    this.shinua = shinua;
    addRequirements(intake, shinua);
  }

  @Override
  public void execute() {
    intake.setDutyIntake(-1);

    shinua.setDutyIndexerOnTop(-1);
    shinua.setDutyIndexerClose(-0.3);
    shinua.setDutyIndexerFar(-0.3);
    
  }

  @Override
  public void end(boolean interrupted) {
    intake.stopIntake(); 
    shinua.stop();
  }

  @Override
  public boolean isFinished() {
    return !triggerToDisable.getAsBoolean();
  }
}