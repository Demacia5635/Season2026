// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.climb.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.climb.constants.ClimbConstants;
import frc.robot.climb.subsystems.Climb;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class StateBasedClimb extends Command {
  /** Creates a new StateBasedClimb. */
  Climb climb;

  public StateBasedClimb(Climb climb) {
    this.climb = climb;
    addRequirements(climb);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (climb.getState()) {
      case IDLE:
        climb.stopArms();
        climb.stopLever();
        break;
      case PREP_CLIMB:
        climb.setArmsAngle(ClimbConstants.ANGLE_ARMS_RAISED);
        climb.setLeverAngle(ClimbConstants.ANGLE_LEVER_CLOSE);
        if (climb.getArmsAngle() >= ClimbConstants.ANGLE_ARMS_RAISED) {
          climb.stopArms();
        }
        if (climb.getAngleLever() >= ClimbConstants.ANGLE_LEVER_CLOSE - ClimbConstants.CLOSE_LEVER_TOLERANCE) {
          climb.stopLever();
        }
        break;
      case CLIMB:
        
        break;
      case GET_OFF_CLIMB:

        break;
      case TESTING:

        break;
      case CLOSE:

        break;
    }
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
