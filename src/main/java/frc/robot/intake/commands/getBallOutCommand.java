// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.utils.controller.CommandController;
import frc.robot.intake.subsystems.IntakeSubsystem;
import frc.robot.intake.subsystems.ShinuaSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class getBallOutCommand extends Command {
  /** Creates a new getBallOutCommand. */
  IntakeSubsystem intake;
  CommandController controller;  
  ShinuaSubsystem shinua;

  public getBallOutCommand(IntakeSubsystem intake, ShinuaSubsystem shinua, CommandController controller) {
    this.intake = intake;
    this.controller = controller;
    this.shinua = shinua;
    addRequirements(intake, shinua);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setDutyIntake(-1);
    shinua.setDutyIndexerOnTop(-1);
    shinua.setDutyIndexerClose(-0.3);
    
    shinua.setDutyIndexerFar(-0.3);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntake(); 
    shinua.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !controller.rightButton().getAsBoolean();
  }
}