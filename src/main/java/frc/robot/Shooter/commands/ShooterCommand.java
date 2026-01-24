// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter.commands;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.utils.chassis.Chassis;
import frc.demacia.utils.controller.CommandController;
import frc.robot.Shooter.subsystem.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShooterCommand extends Command {
  /** Creates a new shooterCommand. */

  Chassis chassis;
  Shooter shooter;
  double vel = 0;
  double hoodAngle = 0;
  CommandController controller;

  public ShooterCommand(Shooter shooter, Chassis chassis) {
    this.shooter = shooter;
    this.chassis = chassis;
    addRequirements(shooter);
    SmartDashboard.putData(this);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Flywheel vel", () -> vel, (x) -> vel = x);
    builder.addDoubleProperty("Hood Angle", () -> hoodAngle, (x) -> hoodAngle = x);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    shooter.setHoodAngle(Math.toRadians(hoodAngle));
    shooter.setFlywheelVel(vel);

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
