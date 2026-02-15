// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.leds;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.utils.leds.LedManager;
import frc.demacia.utils.leds.LedStrip;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LedComand extends Command {
  /** Creates a new LedComand. */

  LedManager ledManger;
  LedStrip ledStrip;

  public LedComand(LedManager ledManager) {
    ledStrip = new LedStrip(getName(), 0, ledManager, 0);
    addRequirements(ledManger);
    // Use addRequirements() here to declare subsystem dependencies.
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ledManger.setBlink(ledStrip, Color.kPowderBlue);
  }

}