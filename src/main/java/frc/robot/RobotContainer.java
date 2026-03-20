// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// bft-pgmc-wgo
package frc.robot;

import static frc.robot.Constants.*;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.demacia.utils.controller.CommandController;
import frc.demacia.utils.controller.CommandController.ControllerType;
import frc.demacia.utils.leds.LedManager;
import frc.demacia.utils.log.LogManager;
import frc.demacia.utils.motors.TalonFXConfig;
import frc.demacia.utils.motors.TalonFXMotor;
import frc.demacia.utils.motors.BaseMotorConfig.Canbus;
import frc.demacia.vision.Camera;
import frc.demacia.vision.subsystem.Dvirs_ObjectPose;
import frc.robot.RobotCommon.RobotStates;
import frc.robot.Shooter.commands.FlywheelPower;
import frc.robot.Shooter.commands.FlywheelTesting;
import frc.robot.Shooter.commands.HoodErrorTesting;
import frc.robot.Shooter.commands.HoodTesting;
import frc.robot.Shooter.commands.ShooterCommand;
import frc.robot.Shooter.commands.ShooterTesting;
import frc.robot.Shooter.subsystem.Shooter;
import frc.robot.Turret.Turret;
import frc.robot.Turret.TurretCommands.TurretCalibration;
import frc.robot.Turret.TurretCommands.TurretCommand;
import frc.robot.Turret.TurretCommands.TurretFollow;
import frc.robot.Turret.TurretCommands.TurretPower;
import frc.robot.buttons.Buttons;
import frc.robot.buttons.ButtonsConstants;
import frc.demacia.odometry.RobotPose;
import frc.demacia.utils.chassis.Chassis;
import frc.demacia.utils.chassis.DriveCommand;
import frc.robot.chassis.RobotAChassisConstants;
import frc.robot.chassis.RobotBChassisConstants;
import frc.robot.chassis.commands.SetModuleAngle;
import frc.robot.intake.commands.IntakeCommand;
import frc.robot.intake.commands.ShinuaCommand;
import frc.robot.intake.commands.getBallOutCommand;
import frc.robot.intake.subsystems.IntakeSubsystem;
import frc.robot.intake.subsystems.ShinuaSubsystem;
import frc.robot.leds.DianasourLedStrip;
import frc.robot.leds.RobotBLedStrip;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer implements Sendable {

  CommandController driverController = new CommandController(0, ControllerType.kPS5);
  public static final PowerDistribution PDH = new PowerDistribution(16, ModuleType.kRev);
  
  public static Chassis chassis;
  public static Turret turret;
  public static IntakeSubsystem intake;
  public static ShinuaSubsystem shinua;
  public static Shooter shooter;
  public static LedManager ledManager;
  public static RobotBLedStrip mainLeds;
  // public static DianasourLedStrip dianasourLedStrip;
  public static Buttons buttons;

  // The robot's subsystems and commands are defined here...

  // Replace with CommandPS4Controller or CommandJoystick if needed

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
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

    PDH.setSwitchableChannel(true);
    
    StateManager.initialize(chassis, intake, shinua, turret, shooter, driverController, mainLeds);

    SmartDashboard.putData("RC", this);
    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
    SmartDashboard.putData("Check Electronics", new InstantCommand(() -> {
      chassis.checkElectronics();
      intake.checkElectronics();
      shinua.checkElectronics();
      turret.checkElectronics();
      shooter.checkElectronics();
    }).ignoringDisable(true));
    // addStatesToElasticForTesting();
    configureBindings();
    setUserButton();
    // SmartDashboard.putNumber("ball angle", ballCamera.getYaw());
    // SmartDashboard.putNumber("ball dist", ballCamera.getDistance());

    // Data.setFrequancyAll();

    configureAuto();
    SmartDashboard.putData("reconfigure auto", new InstantCommand(this::configureAuto).ignoringDisable(true));
    SmartDashboard.putData("PDH", PDH);
  }

  private AutoFactory autoFactory;
  private Command autoCommand;
  private Timer autoTimer = new Timer();

  private void configureAuto() {
    autoFactory = new AutoFactory(() -> RobotCommon.currentRobotPose, (pose) -> RobotPose.getInstance().resetPose(pose),
        chassis::followTrajectory,
        RobotCommon.isRed, chassis);

    AutoRoutine autoRoutine = autoFactory.newRoutine("Balls");

    AutoTrajectory startToBalls = autoRoutine.trajectory("StartToBalls");
    AutoTrajectory ballsToDepot = autoRoutine.trajectory("BallsToDepot");
    AutoTrajectory ballsToShoot = autoRoutine.trajectory("BallsToShoot");
    AutoTrajectory shootToBalls2 = autoRoutine.trajectory("ShootToBalls2");
    AutoTrajectory balls2ToShoot = autoRoutine.trajectory("Balls2ToShoot");
    AutoTrajectory balls2ToDepot = autoRoutine.trajectory("Balls2ToDepot");
    AutoTrajectory shootToBalls3 = autoRoutine.trajectory("ShootToBalls3");
    AutoTrajectory balls3ToShoot = autoRoutine.trajectory("Balls3ToShoot");
    AutoTrajectory balls3ToDepot = autoRoutine.trajectory("Balls3ToDepot");
    AutoTrajectory shootToDepot = autoRoutine.trajectory("ShootToDepot");
    AutoTrajectory depotToTower = autoRoutine.trajectory("DepotToTower");
    AutoTrajectory shootToTower = autoRoutine.trajectory("ShootToTower");

    Trigger timeToDepot = new Trigger(() -> DriverStation.getMatchTime() != 0);
    Trigger timeToBalls2 = new Trigger(() -> DriverStation.getMatchTime() != 0);
    Trigger timeToBalls2FromShoot = new Trigger(() -> DriverStation.getMatchTime() != 0);
    Trigger timeToBalls1DepotFromShoot = new Trigger(() -> DriverStation.getMatchTime() != 0);
    Trigger timeToBalls3 = new Trigger(() -> DriverStation.getMatchTime() != 0);
    Trigger timeToBalls2Depot = new Trigger(() -> DriverStation.getMatchTime() != 0);
    Trigger timeToBalls3FromShoot = new Trigger(() -> DriverStation.getMatchTime() != 0);
    Trigger timeToDepotFromShoot = new Trigger(() -> DriverStation.getMatchTime() != 0);
    Trigger timeToBalls3Depot = new Trigger(() -> DriverStation.getMatchTime() != 0);

    double timeToWaitForShooting = 1.3;

    autoRoutine.active().onTrue(
        Commands.sequence(
            new InstantCommand(() -> {
              autoTimer.reset();
              autoTimer.start();
              chassis.resetTrajectory();
              StateManager.getInstance().setStateChangeActivated(false);
              RobotCommon.changeState(RobotStates.Drive);
              // CommandScheduler.getInstance().schedule(new IntakeCommand(intake));
              // CommandScheduler.getInstance().schedule(new ShinuaCommand(shinua));
              // CommandScheduler.getInstance().schedule(new TurretCommand(turret));
              // CommandScheduler.getInstance().schedule(new ShooterCommand(shooter,
              // chassis));
            }, chassis),
            new WaitCommand(2),
            // startToBalls.resetOdometry(),
            startToBalls.cmd(),
            new RunCommand(() -> chassis.setVelocities(new ChassisSpeeds(0, 0, 0)), chassis)
        ));

        startToBalls.doneDelayed(0).onTrue(ballsToShoot.cmd());
        ballsToShoot.doneDelayed(2).onTrue(shootToBalls2.cmd());
        shootToBalls2.doneDelayed(0).onTrue(balls2ToDepot.cmd());
        // startToBalls.done().and(timeToBalls2.or(timeToDepot.negate())).onTrue(ballsToShoot.cmd());
    // startToBalls.done().and(timeToDepot).and(timeToBalls2.negate()).onTrue(ballsToDepot.cmd());
    // ballsToShoot.doneDelayed(timeToWaitForShooting).and(timeToBalls2FromShoot).onTrue(shootToBalls2.cmd());
    // ballsToShoot.doneDelayed(timeToWaitForShooting).and(timeToBalls1DepotFromShoot.and(timeToBalls2.negate()))
    // .onTrue(shootToDepot.cmd());
    // ballsToShoot.doneDelayed(timeToWaitForShooting).and(timeToBalls2FromShoot.negate()
    // .and(timeToBalls1DepotFromShoot.negate()))
    // .onTrue(shootToTower.cmd());
    // shootToBalls2.done().and(timeToBalls3.or(timeToBalls2Depot.negate())).onTrue(balls2ToShoot.cmd());
    // shootToBalls2.done().and(timeToBalls2Depot.and(timeToBalls3.negate())).onTrue(balls2ToDepot.cmd());
    // balls2ToShoot.doneDelayed(timeToWaitForShooting).and(timeToBalls3FromShoot).onTrue(shootToBalls3.cmd());
    // balls2ToShoot.doneDelayed(timeToWaitForShooting).and(timeToDepotFromShoot.and(timeToBalls2FromShoot.negate()))
    // .onTrue(shootToDepot.cmd());
    // balls2ToShoot.doneDelayed(timeToWaitForShooting).and(timeToBalls3FromShoot.negate()
    // .and(timeToDepotFromShoot.negate())).onTrue(shootToTower.cmd());
    // shootToBalls3.done().and(timeToBalls3Depot).onTrue(balls3ToDepot.cmd());
    // shootToBalls3.done().and(timeToBalls3Depot.negate()).onTrue(balls3ToShoot.cmd());
    // balls3ToDepot.done().onTrue(depotToTower.cmd());
    // balls3ToShoot.doneDelayed(timeToWaitForShooting).onTrue(shootToTower.cmd());
    // shootToDepot.done().onTrue(depotToTower.cmd());
    // balls2ToDepot.done().onTrue(depotToTower.cmd());

    autoRoutine.anyDone(shootToTower, depotToTower).onTrue(
        Commands.sequence(
            new WaitCommand(3),
            new InstantCommand(() -> autoTimer.stop()),
            new RunCommand(() -> chassis.setModuleState(new SwerveModuleState(0d, Rotation2d.kZero)), chassis)));

    autoCommand = autoRoutine.cmd();
  }

  
  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    chassis.setDefaultCommand(new DriveCommand(chassis, driverController));
    intake.setDefaultCommand(new IntakeCommand(intake));
    shinua.setDefaultCommand(new ShinuaCommand(shinua));
    driverController.rightButton().whileTrue(new getBallOutCommand(intake, driverController));
    shooter.setDefaultCommand(new ShooterCommand(shooter, chassis));
    // shooter.setDefaultCommand(new HoodErrorTesting());
    turret.setDefaultCommand(new TurretCommand(turret));
    SmartDashboard.putData("Auto Drive Test", new RunCommand(()->chassis.setRobotRelVelocities(new ChassisSpeeds(-1, 0, 0)), chassis).alongWith(RobotCommon.changeStateCommand(RobotStates.Hub)));
    SmartDashboard.putData("Set Hood angle", new InstantCommand(()->shooter.setHoodMotorPosition(Math.toRadians(86))).ignoringDisable(true));

    buttons.addButton(ButtonsConstants.VOLTS_RANGE[0], new InstantCommand(() -> mainLeds.setColor(Color.kRed)).ignoringDisable(true));
    buttons.addButton(ButtonsConstants.VOLTS_RANGE[1], new InstantCommand(() -> mainLeds.setColor(Color.kYellow)).ignoringDisable(true));
    buttons.addButton(ButtonsConstants.VOLTS_RANGE[2], new InstantCommand(() -> mainLeds.setColor(Color.kGreen)).ignoringDisable(true));
    buttons.addButton(ButtonsConstants.VOLTS_RANGE[3], new InstantCommand(() -> mainLeds.setColor(Color.kBlue)).ignoringDisable(true));
    buttons.addButton(ButtonsConstants.VOLTS_RANGE[4], new InstantCommand(() -> mainLeds.setColor(Color.kOrange)).ignoringDisable(true));

    driverController.rightButton().onTrue(RobotCommon.changeStateCommand(RobotStates.Delivery));
    driverController.downButton().onTrue(new TurretFollow(turret, Field.HubRed.CENTER, chassis));

    driverController.povDown().onTrue(RobotCommon.changeStateCommand(RobotStates.Hub));
    driverController.povUp().onTrue(RobotCommon.changeStateCommand(RobotStates.Drive));
    driverController.rightBumper().onTrue(new InstantCommand(() -> {
      if (StateManager.getInstance().isStateChangeActivated()) {
        StateManager.getInstance().setStateChangeActivated(false);
        RobotCommon.changeState(RobotStates.DriveWithIntake);
      } else {
        StateManager.getInstance().setStateChangeActivated(true);
      }
    }));
    driverController.leftBumper().onTrue(RobotCommon.changeStateCommand(RobotStates.Idle).ignoringDisable(true).andThen(
        new InstantCommand(() -> StateManager.getInstance().setStateChangeActivated(false)).ignoringDisable(true))
        .ignoringDisable(true));

    SmartDashboard.putData("Turret/Calibration", new TurretCalibration(turret));
  }

  private void setUserButton() {
    new Trigger(() -> !DriverStation.isEnabled() &&
        RobotController.getUserButton())
        .onTrue(new SetRobotNeutralMode(chassis, intake, shinua, turret,
            shooter).ignoringDisable(true));
  }

  public Command disableInit() {
    return new InstantCommand(() -> {
      chassis.stop();
      intake.stopIntake();
      shinua.stop();
      turret.stop();
      shooter.stop();
      StateManager.getInstance().resetShift();
    }).withName("Disable Init").ignoringDisable(true);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addBooleanProperty("is comp", () -> RobotCommon.isComp, (isComp) -> RobotCommon.isComp = isComp);
    builder.addBooleanProperty("is red", () -> RobotCommon.isRed, (isRed) -> RobotCommon.isRed = isRed);
    builder.addDoubleProperty("maxDriveVelocity", () -> chassis.getConfig().maxDriveVelocity,
        (max) -> chassis.getConfig().withMaxDriveVelocity(max));

    builder.addBooleanProperty("change is Robot Calibrated for testing", () -> RobotCommon.isRobotCalibrated,
        (isRobotCalibrated) -> RobotCommon.isRobotCalibrated = isRobotCalibrated);
    builder.addDoubleProperty("change Accuracy for testing", () -> RobotCommon.targetAccuracy,
        (targetAccuracy) -> RobotCommon.targetAccuracy = targetAccuracy);

    builder.addDoubleProperty("Match Time", DriverStation::getMatchTime, null);
  }

  public static void updateCommon() {
    Translation2d currentPoseFromHub = RobotCommon.currentRobotPose.getTranslation().minus(HUB_POS);
    RobotCommon.currentDistanceFromTarget = currentPoseFromHub.getNorm();
    RobotCommon.currentAngleFromTarget = currentPoseFromHub.getAngle().getRadians();
    RobotCommon.currentWantedTurretAngle = RobotCommon.currentWantedTurretAngle
        - RobotCommon.currentRobotPose.getRotation().getRadians();

    Translation2d futurePoseFromHub = RobotCommon.futureRobotPose.getTranslation().minus(HUB_POS);
    RobotCommon.futureDistanceFromTarget = futurePoseFromHub.getNorm();
    RobotCommon.futureAngleFromTarget = futurePoseFromHub.getAngle().getRadians();
    RobotCommon.futureWantedTurretAngle = RobotCommon.futureWantedTurretAngle
        - RobotCommon.futureRobotPose.getRotation().getRadians();

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return null;
    return autoCommand;
  }

}