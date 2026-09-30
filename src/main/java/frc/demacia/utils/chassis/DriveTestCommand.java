// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demacia.utils.chassis;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;

import frc.demacia.utils.controller.CommandController;

import frc.robot.RobotCommon;
import frc.robot.Shooter.subsystem.Shooter;

public class DriveTestCommand extends Command {
  private Chassis chassis;
  private CommandController controller;
  private double direction;
  private ChassisSpeeds speeds;
  private static boolean isPrecisionMode;
  private double targetAngle;

  /** Creates a new DriveCommand. */
  public DriveTestCommand(Chassis chassis, CommandController controller) {
    this.chassis = chassis;
    this.controller = controller;
    isPrecisionMode = false;
    addRequirements(chassis);
  }
  @Override
  public void initSendable(SendableBuilder builder) {
      builder.addDoubleProperty("mudole angle", ()-> targetAngle, (x)-> targetAngle =x);
      super.initSendable(builder);
  }
  private void steerByElastic() {
    chassis.setSteerPositions(targetAngle);
    
  }

  public static void setPrecisionMode() {
    isPrecisionMode = !isPrecisionMode;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isPrecisionMode = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    steerByElastic();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
