// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.climb.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.climb.constants.ClimbConstants;
import frc.robot.climb.subsystems.Climb;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimbTower extends Command {
    Climb climb;
  private int currentSpikeCounter = 0;
  private static final double CURRENT_THRESHOLD = 10.0; // Amperes
  private final int CLIMB_CYCLE_TO_STOP = 5;
  private boolean IS_AT_BAR= false;
  /** Creates a new ClimbTower. */
  public ClimbTower(Climb climb) {
    this.climb = climb;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentSpikeCounter = 0;
    IS_AT_BAR = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    lowerArmsToBar();
    if(IS_AT_BAR){
       climb.setLeverAngle(Math.toRadians(ClimbConstants.ANGLE_LEVER_OPEN));
    }
    
  }
  private void lowerArmsToBar() {
     double currentAmper = climb.getCurrentAmpersArms();
    if (currentAmper >= CURRENT_THRESHOLD) {
      currentSpikeCounter++;
    } else {
      currentSpikeCounter = 0; 
    }
    if (currentSpikeCounter >= CLIMB_CYCLE_TO_STOP) { 
          climb.stopArms();
          IS_AT_BAR = true;
    } else {
      climb.setArmsDuty(ClimbConstants.POWER_TO_LOWER_ARMS);
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


