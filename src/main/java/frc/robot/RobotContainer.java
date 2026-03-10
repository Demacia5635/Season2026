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
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import frc.robot.Shooter.commands.HoodTesting;
import frc.robot.Shooter.commands.ShooterCommand;
import frc.robot.Shooter.commands.ShooterTesting;
import frc.robot.Shooter.subsystem.Shooter;
import frc.robot.Turret.Turret;
import frc.robot.Turret.TurretCommands.TurretCalibration;
import frc.robot.Turret.TurretCommands.TurretCommand;
import frc.demacia.odometry.RobotPose;
import frc.demacia.utils.chassis.Chassis;
import frc.demacia.utils.chassis.DriveCommand;
import frc.robot.chassis.RobotAChassisConstants;
import frc.robot.chassis.RobotBChassisConstants;
import frc.robot.chassis.commands.SetModuleAngle;
import frc.robot.climb.commands.StateBasedClimb;
import frc.robot.climb.subsystems.Climb;
import frc.robot.intake.commands.IntakeCommand;
import frc.robot.intake.commands.ShinuaCommand;
import frc.robot.intake.subsystems.IntakeSubsystem;
import frc.robot.intake.subsystems.ShinuaSubsystem;
import frc.robot.leds.RobotALedStrip;

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

  public static Chassis chassis;
  CommandController driverController = new CommandController(0, ControllerType.kPS5);
  public static Turret turret;
  public static IntakeSubsystem intake;
  public static ShinuaSubsystem shinua;
  public static Shooter shooter;
  public static LedManager ledManager;
  public static RobotALedStrip leds;
  public static Climb climb;
  private Dvirs_ObjectPose ballCamera;

  TalonFXMotor motor;
  // The robot's subsystems and commands are defined here...

  // Replace with CommandPS4Controller or CommandJoystick if needed

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    intake = IntakeSubsystem.getInstance();
    shinua = ShinuaSubsystem.getInstance();
    shooter = Shooter.getInstance();
    ledManager = new LedManager();
    leds = new RobotALedStrip();
    climb = new Climb();
    chassis = new Chassis(RobotBChassisConstants.CHASSIS_CONFIG);
    turret = Turret.getInstance();
    ballCamera = new Dvirs_ObjectPose(
        new Camera("balls", new Translation3d(0.27, -0.15, 0.308), -30, 22, false, true));
    StateManager.initalize(chassis, intake, shinua, turret, shooter, driverController, leds);

    SmartDashboard.putData("RC", this);
    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
    SmartDashboard.putData("Check Electronics", new InstantCommand(() -> {
      chassis.checkElectronics();
      intake.checkElectronics();
      shinua.checkElectronics();
      turret.checkElectronics();
      shooter.checkElectronics();
      // climb.checkElectronics();
    }).ignoringDisable(true));
    // addStatesToElasticForTesting();
    configureBindings();
    setUserButton();
    // SmartDashboard.putNumber("ball angle", ballCamera.getYaw());
    // SmartDashboard.putNumber("ball dist", ballCamera.getDistance());

    // Data.setFrequancyAll();

    configureAuto();
    SmartDashboard.putData("reconfigure auto", new InstantCommand(this::configureAuto).ignoringDisable(true));
  }

  private AutoFactory autoFactory;
  private Command autoCommand;
  private Timer autoTimer = new Timer();

  private void configureAuto() {
    autoFactory = new AutoFactory(chassis::getPose, RobotPose.getInstance()::resetPose, chassis::followTrajectory,
        RobotCommon.isRed, chassis);

    AutoRoutine autoRoutine = autoFactory.newRoutine("Balls");

    AutoTrajectory startToBalls = autoRoutine.trajectory("startToBalls");
    AutoTrajectory ballsToDepot = autoRoutine.trajectory("ballsToDepot");
    AutoTrajectory ballsToShoot = autoRoutine.trajectory("ballsToShoot");
    AutoTrajectory shootToBalls2 = autoRoutine.trajectory("shootToBalls2");
    AutoTrajectory balls2ToShoot = autoRoutine.trajectory("balls2ToShoot");
    AutoTrajectory balls2ToDepot = autoRoutine.trajectory("balls2ToDepot");
    AutoTrajectory shootToBalls3 = autoRoutine.trajectory("shootToBalls3");
    AutoTrajectory balls3ToShoot = autoRoutine.trajectory("balls3ToShoot");
    AutoTrajectory balls3ToDepot = autoRoutine.trajectory("balls3ToDepot");
    AutoTrajectory shootToDepot = autoRoutine.trajectory("shootToDepot");
    AutoTrajectory depotToTower = autoRoutine.trajectory("depotToTower");
    AutoTrajectory shootToTower = autoRoutine.trajectory("shootToTower");

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
              CommandScheduler.getInstance().schedule(new IntakeCommand(intake));
              CommandScheduler.getInstance().schedule(new ShinuaCommand(shinua));
              CommandScheduler.getInstance().schedule(new TurretCommand(turret));
              CommandScheduler.getInstance().schedule(new ShooterCommand(shooter, chassis));
              CommandScheduler.getInstance().schedule(new StateBasedClimb(climb, chassis));
            }),
            new WaitCommand(1.1),
            startToBalls.resetOdometry(),
            startToBalls.cmd()));

    startToBalls.done().and(timeToBalls2.or(timeToDepot.negate())).onTrue(ballsToShoot.cmd());
    startToBalls.done().and(timeToDepot).and(timeToBalls2.negate()).onTrue(ballsToDepot.cmd());
    ballsToShoot.doneDelayed(timeToWaitForShooting).and(timeToBalls2FromShoot).onTrue(shootToBalls2.cmd());
    ballsToShoot.doneDelayed(timeToWaitForShooting).and(timeToBalls1DepotFromShoot.and(timeToBalls2.negate()))
        .onTrue(shootToDepot.cmd());
    ballsToShoot.doneDelayed(timeToWaitForShooting).and(timeToBalls2FromShoot.negate()
        .and(timeToBalls1DepotFromShoot.negate()))
        .onTrue(shootToTower.cmd());
    shootToBalls2.done().and(timeToBalls3.or(timeToBalls2Depot.negate())).onTrue(balls2ToShoot.cmd());
    shootToBalls2.done().and(timeToBalls2Depot.and(timeToBalls3.negate())).onTrue(balls2ToDepot.cmd());
    balls2ToShoot.doneDelayed(timeToWaitForShooting).and(timeToBalls3FromShoot).onTrue(shootToBalls3.cmd());
    balls2ToShoot.doneDelayed(timeToWaitForShooting).and(timeToDepotFromShoot.and(timeToBalls2FromShoot.negate()))
        .onTrue(shootToDepot.cmd());
    balls2ToShoot.doneDelayed(timeToWaitForShooting).and(timeToBalls3FromShoot.negate()
        .and(timeToDepotFromShoot.negate())).onTrue(shootToTower.cmd());
    shootToBalls3.done().and(timeToBalls3Depot).onTrue(balls3ToDepot.cmd());
    shootToBalls3.done().and(timeToBalls3Depot.negate()).onTrue(balls3ToShoot.cmd());
    balls3ToDepot.done().onTrue(depotToTower.cmd());
    balls3ToShoot.doneDelayed(timeToWaitForShooting).onTrue(shootToTower.cmd());
    shootToDepot.done().onTrue(depotToTower.cmd());
    balls2ToDepot.done().onTrue(depotToTower.cmd());

    autoRoutine.anyDone(shootToTower, depotToTower).onTrue(
      Commands.sequence(
        new WaitCommand(3),
        new InstantCommand(() -> autoTimer.stop()),
          new RunCommand(() -> chassis.setModuleState(new SwerveModuleState(0d, Rotation2d.kZero)), chassis)
      )
    );

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

    shooter.setDefaultCommand(new ShooterCommand(shooter, chassis));
    // climb.setDefaultCommand(new StateBasedClimb(climb, chassis));
    // driverController.rightButton().onTrue(new ControllerClimb(driverController,
    // climb));
    // climb.setDefaultCommand(new ControllerClimb(driverController, climb));

    // turret.setDefaultCommand(new TurretFollow(turret,
    // Field.HUB(true).getCenter().getTranslation(), chassis));
    SmartDashboard.putData("Activate Feeder", new StartEndCommand(() -> {
      shooter.setFeederPower(0.8);
    }, () -> {
      shooter.setFeederPower(0);
    }));

    // shooter.setDefaultCommand(new ShooterTesting(shooter));
    // turret.setDefaultCommand(new TurretPower(driverController));
    turret.setDefaultCommand(new TurretCommand(turret));
    // turret.setDefaultCommand(new TurretFollow(turret,
    // Field.HUB(true).getCenter().getTranslation(), chassis));

    driverController.downButton().onTrue(RobotCommon.changeStateCommand(RobotStates.HubWithAutoIntake));
    driverController.upButton().onTrue(RobotCommon.changeStateCommand(RobotStates.DriveAutoIntake));
    driverController.rightButton().onTrue(RobotCommon.changeStateCommand(RobotStates.DeliveryWithAutoIntake));

    driverController.povDown().onTrue(RobotCommon.changeStateCommand(RobotStates.HubWithoutAutoIntake));
    driverController.povUp().onTrue(RobotCommon.changeStateCommand(RobotStates.Drive));
    driverController.povRight().onTrue(RobotCommon.changeStateCommand(RobotStates.DeliveryWithoutAutoIntake));
    // new Trigger(() -> driverController.rightBumper().getAsBoolean() ||
    // driverController.getRightTrigger(0.2).getAsBoolean() ||
    // driverController.getLeftTrigger(0.2).getAsBoolean())
    // .onTrue(new InstantCommand(() -> {
    // switch (RobotCommon.currentState) {
    // case DeliveryWithAutoIntake:
    // RobotCommon.changeState(RobotStates.DeliveryWithoutAutoIntake);
    // break;
    // case DeliveryWithoutAutoIntake:
    // RobotCommon.changeState(RobotStates.DeliveryWithAutoIntake);
    // break;
    // case DriveAutoIntake:
    // RobotCommon.changeState(RobotStates.DriveWithIntake);
    // break;
    // case DriveWithIntake:
    // RobotCommon.changeState(RobotStates.DriveAutoIntake);
    // break;
    // case HubWithAutoIntake:
    // RobotCommon.changeState(RobotStates.HubWithoutAutoIntake);
    // break;
    // case HubWithoutAutoIntake:
    // RobotCommon.changeState(RobotStates.HubWithAutoIntake);
    // break;
    // default:
    // break;
    // }
    // }).ignoringDisable(true));
    driverController.rightBumper().onTrue(RobotCommon.changeStateCommand(RobotStates.Idle));

    SmartDashboard.putData("Auto Drive",
        new RunCommand(() -> chassis.setRobotRelVelocities(new ChassisSpeeds(2, 0, 0)), chassis));

    SmartDashboard.putData("Reset pose", new InstantCommand(() -> chassis.resetPose(Pose2d.kZero)));
    // SmartDashboard.putData("lever to zero", new
    // InstantCommand(()->climb.setLeverAngle(0)));
    // SmartDashboard.putData("calibrate Climb", new CalibrateClimb(climb));
    // SmartDashboard.putData("Reset Turret Position",
    // new InstantCommand(() ->
    // turret.setEncoderPosition(0)).ignoringDisable(true));
    SmartDashboard.putData("Turret Calibration", new TurretCalibration(turret));
    SmartDashboard.putData("Config steer", new SetModuleAngle(chassis));
  }

  private void setUserButton() {
    new Trigger(() -> !DriverStation.isEnabled() &&
        RobotController.getUserButton())
        .onTrue(new SetRobotNeutralMode(chassis, intake, shinua, turret,
            shooter).ignoringDisable(true));
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

    builder.addDoubleProperty("Auto Timer", () -> 20 - autoTimer.get(), null);
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
    // return new DuchToBalls(chassis, intake, shinua, turret, shooter, climb);
  }

}