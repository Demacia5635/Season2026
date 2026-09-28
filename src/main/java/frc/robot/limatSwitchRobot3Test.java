// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.logging.LogManager;

import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.utils.chassis.Chassis;
import frc.demacia.utils.sensors.LimitSwitch;
import frc.demacia.utils.sensors.LimitSwitchConfig;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class limatSwitchRobot3Test extends Command {
  /** Creates a new limatSwitchRobot3Test. */

  LimitSwitch limitSwitch;
  LimitSwitchConfig limitSwitchConfig = new LimitSwitchConfig(8, "limate switch");
  public limatSwitchRobot3Test() {
    limitSwitch = new LimitSwitch(limitSwitchConfig);
    addRequirements(Chassis.getInstance());
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    frc.demacia.utils.log.LogManager.log(limitSwitch.get());
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
