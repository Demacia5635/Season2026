// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.climb.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.climb.constants.ClimbConstants;
import frc.robot.climb.subsystems.Climb;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimbPrep extends Command {
  Climb climb;
  Timer openArmsTimer;
  private boolean isArmsOpen;
  private boolean isLeverClosed;

  /** Creates a new OpenArms. */
  public ClimbPrep(Climb climb) {
    this.climb = climb;
    openArmsTimer = new Timer();
    addRequirements(climb);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    openArmsTimer.reset();
    openArmsTimer.start();
    isArmsOpen = false;    
    isLeverClosed = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climb.setArmsDuty(ClimbConstants.POWER_TO_RAISE_ARMS);
    climb.setLeverAngle(ClimbConstants.ANGLE_LEVER_CLOSE);
    if(openArmsTimer.hasElapsed(ClimbConstants.TIME_TO_RAISE_ARMS)){
      isArmsOpen =  true;
      climb.stopArms();
    }
    if(climb.getAngleLever()>= ClimbConstants.ANGLE_LEVER_CLOSE-ClimbConstants.CLOSE_LEVER_TOLERANCE){
      isLeverClosed = true;
      climb.stopLever();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climb.stopArms();
    climb.stopLever();
    openArmsTimer.stop();
    openArmsTimer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isArmsOpen && isLeverClosed;
  }
}
