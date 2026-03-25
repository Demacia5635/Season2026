// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Turret.TurretCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Turret.Turret;

public class TurretCalibration extends Command {

  private final Turret turret;
  
  public TurretCalibration(Turret turret) {
    this.turret = turret;
    addRequirements(turret);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    turret.setPower(0.1);

  }

  @Override
  public void end(boolean interrupted) {
    turret.stop();
    if (!interrupted) {
      turret.setCalibrated();
      turret.updatePositionByLimit();
    }
  }

  @Override
  public boolean isFinished() {
    return turret.isAtMaxLimit() || turret.isAtMinLimit();
  }
}
