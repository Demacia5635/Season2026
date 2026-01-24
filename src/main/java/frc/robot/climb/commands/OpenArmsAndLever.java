// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.climb.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.climb.constants.ClimbConstants;
import frc.robot.climb.subsystems.Climb;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class OpenArmsAndLever extends Command {
  Climb climb;
  /** Creates a new OpenArms. */
  public OpenArmsAndLever(Climb climb) {
    this.climb = climb;
    addRequirements(climb);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climb.setArmsAngle(Math.toRadians(ClimbConstants.ANGLE_ARMS_OPEN));
    climb.setLeverAngle(Math.toRadians(ClimbConstants.ANGLE_LEVER_OPEN));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climb.stopArms();
    climb.stopLever();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return climb.getPositionArms() >= Math.toRadians(ClimbConstants.ANGLE_ARMS_OPEN)-0.1 && climb.getPositionLever() >= Math.toRadians(ClimbConstants.ANGLE_LEVER_OPEN)-0.1;
  }
}
