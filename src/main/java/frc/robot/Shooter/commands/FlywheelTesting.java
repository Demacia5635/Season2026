// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.utils.controller.CommandController;
import frc.robot.Shooter.subsystem.Shooter;

/** 
 * this command is a fly weel tsting
 * for test the vel of the fly weel
*/
public class FlywheelTesting extends Command {
  Shooter shooter;
  CommandController controller;
  double wantedVel = 0;

  public FlywheelTesting(Shooter shooter, CommandController controller) {
    this.shooter = shooter;
    this.controller = controller;
    addRequirements(shooter);
    SmartDashboard.putData("Flywheel testing", this);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  /**
   * @param wantedVel
   * a viable that cange the vel
   * we set it in the elastic
   */

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Wanted flywheel vel", () -> wantedVel, (x) -> wantedVel = x);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  /**
   * in the execute we set the vel
   * by set the vaubole to the funcsan setFlyWeelVel()
   * he is set in the subsystem
   */

  // Called every time the scheduler runs while the command is scheduled.
  public void execute() {
    shooter.setFlywheelVel(wantedVel);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
