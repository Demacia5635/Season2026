// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.climb.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.climb.constants.ClimbConstants;
import frc.robot.climb.subsystems.Climb;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GetOffTower extends Command {
  /** Creates a new GetOffTower. */
  Climb climb;
  Timer openArmsTimerAfterClimb;
  private boolean IS_LEVER_CLOSED;
  public GetOffTower(Climb climb) {
    this.climb = climb;
    openArmsTimerAfterClimb = new Timer();

    addRequirements(climb);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    IS_LEVER_CLOSED = false;
    openArmsTimerAfterClimb.stop();
    openArmsTimerAfterClimb.reset();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climb.setLeverDuty(ClimbConstants.POWER_TO_CLOSE_LEVER);
    if (!IS_LEVER_CLOSED && climb.getAngleLever() >= ClimbConstants.ANGLE_LEVER_CLOSE ) {
      climb.stopLever();
      IS_LEVER_CLOSED = true;
      openArmsTimerAfterClimb.start();

    }
    if (IS_LEVER_CLOSED){
      climb.setArmsDuty(ClimbConstants.POWER_TO_RAISE_ARMS);
    }
      if(openArmsTimerAfterClimb.hasElapsed(ClimbConstants.TIME_TO_RAISE_ARMS_AFTER_CLIMB)){
        climb.stopArms();//need to think how to close the arms after raising them(code review)
      }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climb.stopArms();
    climb.stopLever();
    openArmsTimerAfterClimb.stop();
    openArmsTimerAfterClimb.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; 
  }
}
