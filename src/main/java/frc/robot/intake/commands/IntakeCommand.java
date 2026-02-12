// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotCommon;
import frc.robot.intake.IntakeConstants;
import frc.robot.intake.subsystems.IntakeSubsystem;
import frc.robot.intake.subsystems.ShinuaSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCommand extends Command {

  private double batteryPower;
  private IntakeSubsystem intakeSubsystem;
  private ShinuaSubsystem shinuaSubsystem;

  /** Creates a new IntakeCommand. */
  public IntakeCommand(IntakeSubsystem intakeSubsystem, ShinuaSubsystem shinuaSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.shinuaSubsystem = shinuaSubsystem;
    batteryPower = IntakeConstants.MAX_POWER;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem, shinuaSubsystem);
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

        //indexers
        shinuaSubsystem.setDutyIndexerOnTop(IntakeConstants.MAX_POWER);
        shinuaSubsystem.setDutyIndexerClose(IntakeConstants.MAX_POWER);
        shinuaSubsystem.setDutyIndexerFar(IntakeConstants.MAX_POWER);

        //battery
        // if (shinuaSubsystem.isAtMax(Math.toRadians(15))) {
        //   batteryPower = -IntakeConstants.MAX_POWER;
        // } else if(shinuaSubsystem.isAtMin(Math.toRadians(15))){
        //   batteryPower = IntakeConstants.MAX_POWER;
        // }
        // shinuaSubsystem.setPowerBattery(batteryPower);
        break;

      case DriveWhileIntake:
        //intake
        intakeSubsystem.setDutyIntake(IntakeConstants.MAX_POWER);

        //indexers
        shinuaSubsystem.setDutyIndexerOnTop(-IntakeConstants.MAX_POWER);
        shinuaSubsystem.setDutyIndexerClose(IntakeConstants.MAX_POWER);
        shinuaSubsystem.setDutyIndexerFar(-IntakeConstants.MAX_POWER);

        //battery
        // shinuaSubsystem.setPositionBattery(0);
        break;

      case ShootWithoutIntake:
        //intake
        intakeSubsystem.stopIntake();

        //indexers
        shinuaSubsystem.setDutyIndexerOnTop(IntakeConstants.MAX_POWER);
        shinuaSubsystem.setDutyIndexerClose(IntakeConstants.MAX_POWER);
        shinuaSubsystem.setDutyIndexerFar(IntakeConstants.MAX_POWER);

        //battery
        // if (shinuaSubsystem.isAtMax(Math.toRadians(15))) {
        //   batteryPower = -IntakeConstants.MAX_POWER;
        // } else if(shinuaSubsystem.isAtMin(Math.toRadians(15))){
        //   batteryPower = IntakeConstants.MAX_POWER;
        // }
        // shinuaSubsystem.setPowerBattery(batteryPower);
        break;
      
      default:
        intakeSubsystem.stopIntake();
        shinuaSubsystem.stopIndexerOnTop();
        shinuaSubsystem.stopIndexerClose();
        shinuaSubsystem.stopIndexerFar();
        // shinuaSubsystem.setPositionBattery(0);
    }
  }
}
