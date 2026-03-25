// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Turret.TurretCommands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.demacia.utils.controller.CommandController;

import frc.robot.Turret.Turret;

public class TurretPower extends Command {

  private final Turret turret;
  private final CommandController controller;

  public TurretPower(Turret turret, CommandController controller) {
    this.turret = turret;
    this.controller = controller;
    addRequirements(turret);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    turret.setPower(controller.getLeftY() * 0.8);
  }

  @Override
  public void end(boolean interrupted) {
    turret.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
