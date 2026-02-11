// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotCommon;
import frc.robot.intake.IntakeConstants;
import frc.robot.intake.subsystems.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCommand extends Command {

  private IntakeSubsystem intakeSubsystem;

  /** Creates a new IntakeCommand. */
  public IntakeCommand(IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (RobotCommon.currentState) { //ShootWithIntake, ShootWithoutIntake, DriveWhileIntake, Drive, PrepareClimb, Climb, GetOffClimb
      case ShootWithIntake:
        //intake
        intakeSubsystem.setDutyIntake(IntakeConstants.MAX_POWER);

        //indexer on top
        intakeSubsystem.setDutyIndexerOnTop(IntakeConstants.MAX_POWER);

        //indexer close
        intakeSubsystem.setDutyIndexerClose(IntakeConstants.MAX_POWER);

        //indexer far
        intakeSubsystem.setDutyIndexerFar(IntakeConstants.MAX_POWER);

        //battery
        if (intakeSubsystem.isAtMax(Math.toRadians(15))) {
          intakeSubsystem.setPower(-IntakeConstants.MAX_POWER);
        } else if(intakeSubsystem.isAtMin(Math.toRadians(15))){
          intakeSubsystem.setPower(IntakeConstants.MAX_POWER);
        }

        break;

      case DriveWhileIntake:
        //intake
        intakeSubsystem.setDutyIntake(IntakeConstants.MAX_POWER);

        //indexer on top
        intakeSubsystem.setDutyIndexerOnTop(-IntakeConstants.MAX_POWER);

        //indexer close
        intakeSubsystem.setDutyIndexerClose(IntakeConstants.MAX_POWER);

        //indexer far
        intakeSubsystem.setDutyIndexerFar(-IntakeConstants.MAX_POWER);

        //battery
        intakeSubsystem.setPositionBattery(0);
        break;

      case ShootWithoutIntake:
        //intake
        intakeSubsystem.stopIntake();

        //indexer on top
        intakeSubsystem.setDutyIndexerOnTop(IntakeConstants.MAX_POWER);

        //indexer close
        intakeSubsystem.setDutyIndexerClose(IntakeConstants.MAX_POWER);

        //indexer far
        intakeSubsystem.setDutyIndexerFar(IntakeConstants.MAX_POWER);

        //battery
        if (intakeSubsystem.isAtMax(Math.toRadians(15))) {
          intakeSubsystem.setPower(-IntakeConstants.MAX_POWER);
        } else if(intakeSubsystem.isAtMin(Math.toRadians(15))){
          intakeSubsystem.setPower(IntakeConstants.MAX_POWER);
        }
        break;
      
      case GetOffClimb:
        intakeSubsystem.stopIntake();
        intakeSubsystem.stopIndexerOnTop();
        intakeSubsystem.stopIndexerClose();
        intakeSubsystem.stopIndexerFar();
        intakeSubsystem.setPositionBattery(0);
        break;
    }
  }
}
