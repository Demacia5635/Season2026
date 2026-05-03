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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
import frc.robot.Shooter.commands.ShooterCommand;
import frc.robot.Shooter.subsystem.Shooter;
import frc.robot.Turret.Turret;
import frc.robot.Turret.TurretCommands.TurretCalibration;
import frc.robot.Turret.TurretCommands.TurretCommand;
import frc.robot.buttons.Buttons;
import frc.robot.buttons.ButtonsConstants;
import frc.robot.chassis.RobotBChassisConstants;
import frc.robot.intake.commands.IntakeCommand;
import frc.robot.intake.commands.ShinuaCommand;
import frc.robot.intake.commands.GetBallOutCommand;
import frc.robot.intake.subsystems.IntakeSubsystem;
import frc.robot.intake.subsystems.ShinuaSubsystem;
import frc.robot.leds.RobotBLedStrip;

/* TODO: add doc */
public class RobotContainer implements Sendable {

  private static CommandController driverController;
  private static PowerDistribution PDH;

  private static Chassis chassis;
  private static IntakeSubsystem intake;
  private static ShinuaSubsystem shinua;
  private static Turret turret;
  private static Shooter shooter;

  private static LedManager ledManager;
  private static RobotBLedStrip mainLeds;
  // private static DianasourLedStrip dianasourLedStrip;
  private static Buttons buttons;

  public static RobotContainer instance;
  private AutoFactory autoFactory;
  private Command autoCommand;
  private MotorTesting motorTesting;
  private motorTestingCommand motorTestingCommand;
  public RobotContainer() {
    instance = this;
    motorTesting = new MotorTesting();
    motorTesting.setDefaultCommand(motorTestingCommand = new motorTestingCommand(motorTesting));
    driverController = new CommandController(0, ControllerType.kPS5);
    PDH = new PowerDistribution(16, ModuleType.kRev);
    PDH.setSwitchableChannel(true);

    // configureSubsystems();
    // configureUserButton();
    // configureBindings();
    // configureAuto();

    SmartDashboard.putData("RC", this);
    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
    SmartDashboard.putData("PDH", PDH);
    SmartDashboard.putData("reconfigure auto", new InstantCommand(this::configureAuto).ignoringDisable(true));
  }

  private void configureSubsystems() {
    Chassis.initialize(RobotBChassisConstants.CHASSIS_CONFIG);
    chassis = Chassis.getInstance();
    intake = IntakeSubsystem.getInstance();
    shinua = ShinuaSubsystem.getInstance();
    turret = Turret.getInstance();
    shooter = Shooter.getInstance();

    ledManager = new LedManager();
    mainLeds = new RobotBLedStrip();

    // dianasourLedStrip = new DianasourLedStrip();
    buttons = Buttons.getInstance();

    SmartDashboard.putData("Check Electronics", new InstantCommand(() -> {
      chassis.checkElectronics();
      intake.checkElectronics();
      shinua.checkElectronics();
      turret.checkElectronics();
      shooter.checkElectronics();
    }).ignoringDisable(true));
  }

  private void configureUserButton() {
    new Trigger(() -> !DriverStation.isEnabled() &&
        RobotController.getUserButton())
        .onTrue(new SetRobotNeutralMode(chassis, intake, shinua, turret,
            shooter, mainLeds).ignoringDisable(true));
  }

  private void configureAuto() {
    autoFactory = new AutoFactory(RobotCommon::getCurrentRobotPose, (pose) -> RobotPose.getInstance().resetPose(pose),
        chassis::followTrajectory,
        RobotCommon.isRed(), chassis);
  }

  @SuppressWarnings("unused")
  private AutoRoutine soloAuto() {
    AutoRoutine routine = autoFactory.newRoutine("soloRoutine");

    AutoTrajectory trajectory = routine.trajectory("mainAuto/DeliveryTesting");
    // AutoTrajectory trajectory = routine.trajectory("testPath/TestPath");

    routine.active().onTrue(
        Commands.sequence(
            new InstantCommand(() -> {
              chassis.resetTrajectory();
              StateManager.getInstance().setStateChangeActivated(false);
              RobotCommon.changeStateCommand(RobotStates.Trench);
              CommandScheduler.getInstance().schedule(new IntakeCommand(intake));
              CommandScheduler.getInstance().schedule(new ShinuaCommand(shinua));
              // CommandScheduler.getInstance().schedule(new TurretCommand(turret));
              CommandScheduler.getInstance().schedule(new ShooterCommand(shooter));
            }, chassis),
            trajectory.cmd()));

    trajectory.atTime("Delivery").onTrue(RobotCommon.changeStateCommand(RobotStates.Delivery));
    trajectory.atTime("Trench").onTrue(RobotCommon.changeStateCommand(RobotStates.Trench));
    trajectory.atTime("Hub").onTrue(RobotCommon.changeStateCommand(RobotStates.Hub));
    trajectory.atTime("DriveWithIntake").onTrue(RobotCommon.changeStateCommand(RobotStates.DriveWithIntake));

    return routine;
  }

  private void configureBindings() {
    chassis.setDefaultCommand(new DriveCommand(chassis, driverController));
    intake.setDefaultCommand(new IntakeCommand(intake));
    shinua.setDefaultCommand(new ShinuaCommand(shinua));
    shooter.setDefaultCommand(new ShooterCommand(shooter));
    // turret.setDefaultCommand(new TurretCommand(turret));

    /*
     * TODO: change buttons:
     * 1: manual calibration to Turret
     * 2: manual calibration to Hood
     * 3: manual reset gyro {
     * - set camera to 3D
     * - update gyro angle
     * - set camera to 2D
     * - update quest pose
     * }
     * 4: only swerve coast / brake
     * 5: only turret coast / brake
     */

    buttons.addButton(ButtonsConstants.VOLTS_RANGE[0],
        new InstantCommand(() -> {
          mainLeds.startUserButton();
        }).ignoringDisable(true));
    buttons.addButton(ButtonsConstants.VOLTS_RANGE[1],
        new InstantCommand(() -> mainLeds.setColor(Color.kYellow)).ignoringDisable(true));
    buttons.addButton(ButtonsConstants.VOLTS_RANGE[2],
        new InstantCommand(() -> mainLeds.setColor(Color.kGreen)).ignoringDisable(true));
    buttons.addButton(ButtonsConstants.VOLTS_RANGE[3],
        new InstantCommand(() -> mainLeds.setColor(Color.kBlue)).ignoringDisable(true));
    buttons.addButton(ButtonsConstants.VOLTS_RANGE[4],
        new InstantCommand(() -> mainLeds.setColor(Color.kOrange)).ignoringDisable(true));

    driverController.upButton().onTrue(RobotCommon.changeStateCommand(RobotStates.DriveWithIntake));
    driverController.rightBumper().onTrue(
        new InstantCommand(() -> StateManager.getInstance().setStateChangeActivated(true)).ignoringDisable(true));
    driverController.leftBumper().onTrue(RobotCommon.changeStateCommand(RobotStates.Idle));
    driverController.rightButton().onTrue(new GetBallOutCommand(intake, shinua, driverController.rightButton()));
    driverController.downButton().whileTrue(
        new RunCommand(() -> rumble.setRumble(RumbleType.kBothRumble, 1)).withTimeout(0.5).ignoringDisable(true));
    driverController.leftButton().onTrue(new InstantCommand(DriveCommand::setPrecisionMode).ignoringDisable(true));
    driverController.povDown().onTrue(new RunCommand(()->chassis.setVelocities(new ChassisSpeeds(0, 2, 5)), chassis));

    SmartDashboard.putData("Turret/Calibration", new TurretCalibration(turret));
  }

  PS5Controller rumble = new PS5Controller(1);

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addBooleanProperty("is Red", RobotCommon::isRed, RobotCommon::setRed);
    builder.addBooleanProperty("is Comp", RobotCommon::isComp, RobotCommon::setComp);
    builder.addDoubleProperty("match Time", DriverStation::getMatchTime, null);
  }

  public void disableInit() {
    if (chassis != null) chassis.stop();
    if (intake != null) intake.stopIntake();
    if (shinua != null) shinua.stop();
    if (turret != null) turret.stop();
    if (shooter != null) shooter.stop();
    
  
    StateManager.getInstance().resetShift();
  }

  public void periodic() {
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

  public static IntakeSubsystem getIntake() {
    return intake;
  }

  public static ShinuaSubsystem getShinua() {
    return shinua;
  }

  public static Turret getTurret() {
    return turret;
  }

  public static Shooter getShooter() {
    return shooter;
  }

  public static LedManager getLedManager() {
    return ledManager;
  }

  public static RobotBLedStrip getMainLeds() {
    return mainLeds;
  }

  public static Buttons getButtons() {
    return buttons;
  }

  public static RobotContainer getInstance() {
    return instance;
  }

  public AutoFactory getAutoFactory() {
    return autoFactory;
  }
}