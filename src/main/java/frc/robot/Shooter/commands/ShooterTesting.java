// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.utils.controller.CommandController;
import frc.robot.Shooter.subsystem.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */

/**
 * this command @param ShooterTesting is command for test all the shooter
 * 
 * he take from the elastic the vaubol @param wantedFlyWheelVel is what the vel you want the fly weel to move
 * and he also take the vabulse @param wantedAngle  is what the angle you want the hood to be
 * and also the vubol @param isFeederOn is for set the feeder to move
 *  
 */

public class ShooterTesting extends Command {
  /** Creates a new ShooterTsting. */

  Shooter shooter;

  double wantedFlywheelVel = 0;
  double wantedAngle = 0;
  boolean isFeederOn;

  public ShooterTesting(Shooter shooter) {
    this.shooter = shooter;
    this.isFeederOn = false;
    
    wantedAngle = Math.toDegrees(shooter.getHoodAngleMotor());

    addRequirements(shooter);
    SmartDashboard.putData("Shooter Testing Command", this);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  /**
   * this funcsan is for get from the elastic the @param feeder @param wantedAngle and the @param wantedVel
   */

  @Override
  public void initSendable(SendableBuilder builder) {
    // TODO Auto-generated method stub
    super.initSendable(builder);
    builder.addDoubleProperty("Wanted angle", () -> wantedAngle, (x) -> wantedAngle = x);
    builder.addDoubleProperty("Wanted flywheel vel", () -> wantedFlywheelVel, (x) -> wantedFlywheelVel = x);
    builder.addBooleanProperty("activate feeder", () -> isFeederOn, (value) -> isFeederOn = value);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  /**
   * this funcsan the exeute is for run the coomand every 0.02 secend
   * he set the angle vel and feeder that he get form the elastic
   */

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // shooter.setHoodAngle(Math.toRadians(wantedAngle));
    shooter.setFlywheelVel(wantedFlywheelVel);
    shooter.setFeederPower(isFeederOn ? 0.4 : 0);
  }

  /**
   * this funcsan stop the shooter at the end of the commands
   */

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
