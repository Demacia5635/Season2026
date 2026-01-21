// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.utils.chassis.Chassis;
import frc.robot.Shooter.subsystem.Shooter;
//import frc.robot.Shooter.utils.shooterUtilse;
import frc.robot.Shooter.utils.ShooterUtils;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShooterFollowCommand extends Command {
  /** Creates a new shotingWithMovmentCommand. */

  Shooter shooter;
  Chassis chassis;
  Pose2d target;
  Translation3d robotVelosety;
  public static double VelocityInFucer;

  public ShooterFollowCommand(Shooter shooter, Chassis chassis) {
    this.chassis = chassis;
    this.shooter = shooter;
    this.target = Pose2d.kZero;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation3d shooterVelAsVector = shooter.getVelInVector(shooter.getLookUpTableVel(shooter.getVectorToHubShoter().getNorm()));
    Translation3d shooterFinalVel = shooterVelAsVector.minus(ShooterUtils.GetChassisVelAsVector());
    shooter.setFlywheelVel(shooterFinalVel.getNorm());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
