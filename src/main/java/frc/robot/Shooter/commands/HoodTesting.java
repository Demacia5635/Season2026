// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter.commands;

import java.lang.ModuleLayer.Controller;

import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.utils.controller.CommandController;
import frc.robot.Shooter.subsystem.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */

/**
 * this command is  acommand for the tsting of the hood
*/

public class HoodTesting extends Command {
  Shooter shooter;
  CommandController controller;
  public HoodTesting(Shooter shooter,CommandController controller) {
    this.shooter = shooter;
    this.controller = controller;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  /**
   * in the execute we set the hood power
   * to the conroller * 0.8
   */

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setHoodPower(controller.getLeftY()*0.8);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {} // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
