// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class motorTestingCommand extends Command {
  /** Creates a new motorTestingCommand. */
private MotorTesting motorTesting;
private boolean stallDetected = false;
  public motorTestingCommand(MotorTesting motorTesting) {
    this.motorTesting = motorTesting;
    addRequirements(motorTesting);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Only set motor power if no stall has been detected
    if (!motorTesting.getMotorStall()&&!stallDetected) {
      motorTesting.setMotorPower(0.05);
      stallDetected = true;
    } else {
      // Keep motor at 0 power when stall is detected
      motorTesting.setMotorPower(0);
    }
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
