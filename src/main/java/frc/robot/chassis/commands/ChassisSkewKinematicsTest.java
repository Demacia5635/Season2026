// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chassis.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.utils.chassis.Chassis;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ChassisSkewKinematicsTest extends Command {
  Chassis chassis;
  double wantedOmegaRads = 0;
  double wantedVy = 0;

  public ChassisSkewKinematicsTest() {
    chassis = Chassis.getInstance();
    addRequirements(chassis);  
    SmartDashboard.putData("ChassisTesting", this);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
      super.initSendable(builder);


      builder.addDoubleProperty("wanted omega", ()->wantedOmegaRads, (x)->this.wantedOmegaRads = x);
      builder.addDoubleProperty("wanted vy", ()->wantedVy, (x)->this.wantedVy = x);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    chassis.setVelocities(new ChassisSpeeds(0, wantedVy, wantedOmegaRads));
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
