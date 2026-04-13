// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// bft-pgmc-wgo
package frc.robot;

import java.util.ArrayList;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.demacia.odometry.RobotPose;
import frc.demacia.utils.chassis.Chassis;
import frc.demacia.utils.chassis.DriveCommand;
import frc.demacia.utils.controller.CommandController;
import frc.demacia.utils.controller.CommandController.ControllerType;
import frc.demacia.utils.leds.LedManager;

import frc.robot.RobotCommon.RobotStates;
import frc.robot.chassis.RobotBChassisConstants;

/* TODO: add doc */
public class RobotContainer implements Sendable {

  private static CommandController driverController;
  private static PowerDistribution PDH;

  private static Chassis chassis;

  public static RobotContainer instance;
  private AutoFactory autoFactory;
  private Command autoCommand;

  public RobotContainer() {
    instance = this;

    driverController = new CommandController(0, ControllerType.kPS5);
    PDH = new PowerDistribution(16, ModuleType.kRev);
    PDH.setSwitchableChannel(true);

    configureSubsystems();
    configureUserButton();
    configureBindings();

    SmartDashboard.putData("RC", this);
    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
    SmartDashboard.putData("PDH", PDH);
  }

  private void configureSubsystems() {
    Chassis.initialize(RobotBChassisConstants.CHASSIS_CONFIG);
    chassis = Chassis.getInstance();

    SmartDashboard.putData("Check Electronics", new InstantCommand(() -> {
      chassis.checkElectronics();
    }).ignoringDisable(true));
  }

  private void configureUserButton() {
    new Trigger(() -> !DriverStation.isEnabled() &&
        RobotController.getUserButton())
        .onTrue(new SetRobotNeutralMode(chassis).ignoringDisable(true));
  }

  private void configureBindings() {
    chassis.setDefaultCommand(new DriveCommand(chassis, driverController));
  }


  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addBooleanProperty("is Red", RobotCommon::isRed, RobotCommon::setRed);
    builder.addBooleanProperty("is Comp", RobotCommon::isComp, RobotCommon::setComp);
    builder.addDoubleProperty("match Time", DriverStation::getMatchTime, null);
  }

  public void disableInit() {
    chassis.stop();
  }

  public Command getAutonomousCommand() {
    return autoCommand;
  }

  public static CommandController getDriverController() {
    return driverController;
  }

  public static PowerDistribution getPDH() {
    return PDH;
  }

  public static Chassis getChassis() {
    return chassis;
  }

  public static RobotContainer getInstance() {
    return instance;
  }
}