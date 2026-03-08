// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.logMotor;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.utils.motors.TalonFXConfig;
import frc.demacia.utils.motors.TalonFXMotor;
import frc.demacia.utils.motors.BaseMotorConfig.Canbus;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class logMotorCommnad extends Command {

  double wantedPower;
  /** Creates a new logMotor. */
  public logMotorCommnad() {
    // Use addRequirements() here to declare subsystem dependencies.
  }
  
  @Override
  public void initSendable(SendableBuilder builder) {
      builder.addDoubleProperty("log motor power", () -> wantedPower, (x) -> wantedPower =x);
  }
  TalonFXConfig motorConfig = new TalonFXConfig(4,Canbus.Rio,"hi").withBrake(true);

  TalonFXMotor motor = new TalonFXMotor(motorConfig);

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    motor.setDuty(0.5);
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
