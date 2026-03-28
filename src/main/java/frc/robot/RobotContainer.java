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
import edu.wpi.first.net.WebServer;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.demacia.odometry.RobotPose;
import frc.demacia.path.Trgectory.FollowTrajectory;
import frc.demacia.path.utils.PathPoint;
import frc.demacia.utils.chassis.Chassis;
import frc.demacia.utils.chassis.DriveCommand;
import frc.demacia.utils.controller.CommandController;
import frc.demacia.utils.controller.CommandController.ControllerType;
import frc.demacia.utils.geometry.Pose2dDemacia;
import frc.demacia.utils.geometry.Rotation2dDemacia;
import frc.demacia.utils.log.Elastic;
import frc.demacia.utils.log.Elastic.Notification;
import frc.robot.RobotCommon.RobotStates;
import frc.robot.Shooter.commands.ShooterCommand;
import frc.robot.Shooter.subsystem.Shooter;
import frc.robot.Turret.Turret;
import frc.robot.Turret.TurretCommands.TurretCommand;
import frc.robot.buttons.Buttons;
import frc.robot.buttons.ButtonsConstants;
import frc.robot.buttons.SetRobotNeutralMode;
import frc.robot.chassis.RobotBChassisConstants;
import frc.robot.intake.commands.GetBallOutCommand;
import frc.robot.intake.commands.IntakeCommand;
import frc.robot.intake.commands.ShinuaCommand;
import frc.robot.intake.subsystems.IntakeSubsystem;
import frc.robot.intake.subsystems.ShinuaSubsystem;
import frc.robot.leds.RobotBLedStrip;

/* TODO: add doc on entire project */
public class RobotContainer implements Sendable {

  private static CommandController driverController;
  private static PowerDistribution PDH;

  private Chassis chassis;
  private IntakeSubsystem intake;
  private ShinuaSubsystem shinua;
  private Turret turret;
  private Shooter shooter;

  private static RobotBLedStrip mainLeds;
  // private static DianasourLedStrip dianasourLedStrip;
  private static Buttons buttons;

  public static RobotContainer instance;
  private AutoFactory autoFactory;
  private Command autoCommand;

  public RobotContainer() {
    instance = this;

    driverController = new CommandController(0, ControllerType.kPS5);
    PDH = new PowerDistribution(16, ModuleType.kRev);
    PDH.setSwitchableChannel(true);
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

    configureSubsystems();
    configureUserButton();
    configureBindings();
    configureAuto();

    SmartDashboard.putData("RC", this);
    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
    SmartDashboard.putData("PDH", PDH);
    SmartDashboard.putData("reconfigure auto", new InstantCommand(this::configureAuto).ignoringDisable(true));

    Elastic.sendNotification(new Notification(Elastic.NotificationLevel.INFO, "Robot Code Initalize", ""));
  }

  private void configureSubsystems() {
    Chassis.initialize(RobotBChassisConstants.CHASSIS_CONFIG);
    chassis = Chassis.getInstance();
    intake = IntakeSubsystem.getInstance();
    shinua = ShinuaSubsystem.getInstance();
    turret = Turret.getInstance();
    shooter = Shooter.getInstance();

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
    autoFactory = new AutoFactory(
        () -> new Pose2d(RobotCommon.getCurrentRobotPose().getX(), RobotCommon.getCurrentRobotPose().getY(),
            new Rotation2d(RobotCommon.getCurrentRobotPose().getRotation().getRadians())),
        (pose) -> RobotPose.getInstance().resetPose(
            new Pose2dDemacia(pose.getX(), pose.getY(), new Rotation2dDemacia(pose.getRotation().getRadians()))),
        chassis::followTrajectory,
        RobotCommon.isRed(), chassis);

    // autoCommand = soloAuto().cmd();
    autoCommand = demaciaTrajectoryAuto();
  }

  private Command demaciaTrajectoryAuto() {
    ArrayList<PathPoint> points = new ArrayList<>();
    points.add(PathPoint.kZero);

    // points.add(new PathPoint(new Pose2d(9.7, 0.75, Rotation2d.fromDegrees(90)),1,
    // 1));
    // points.add(new PathPoint(new Pose2d(9.7, 4, Rotation2d.fromDegrees(180)),
    // 0,1));
    // points.add(new PathPoint(new Pose2d(Field.FieldDimensions.LENGTH - 4.35,
    // Field.FieldDimensions.WIDTH - 7.4, Rotation2d.fromDegrees(90)), 1, 0));
    // points.add(new PathPoint(new Pose2d(Field.FieldDimensions.LENGTH - 5.8,
    // Field.FieldDimensions.WIDTH - 7.4, Rotation2d.fromDegrees(90)),2, 1));
    // points.add(new PathPoint(new Pose2d(Field.FieldDimensions.LENGTH - (7.1 -
    // 0.5), Field.FieldDimensions.WIDTH - (5.33 - 0.5),
    // Rotation2d.fromDegrees(180-15)),0.5, 1));
    // points.add(new PathPoint(new Pose2d(Field.FieldDimensions.LENGTH - 8.9,
    // Field.FieldDimensions.WIDTH - 3.4, Rotation2d.fromDegrees(180-54.4)),0.5,
    // 3));
    // points.add(new PathPoint(new Pose2d(8.3, 0.5, Rotation2d.fromRadians(0)),1,
    // 3));
    // points.add(new PathPoint(new Pose2d(Field.FieldDimensions.LENGTH - 2.8,
    // Field.FieldDimensions.WIDTH - 7.4, Rotation2d.fromRadians(0)),2, 0));

    points.add(new PathPoint(new Pose2dDemacia(10.18, 0.9, Rotation2dDemacia.kCCW_90deg), 1, 1));
    points.add(new PathPoint(new Pose2dDemacia(10.1, 3.35, Rotation2dDemacia.k180deg), 1, 0.7));
    points.add(new PathPoint(new Pose2dDemacia(9.3, 3.9, Rotation2dDemacia.fromDegrees(135)), 0.3, 0.4));
    points.add(new PathPoint(new Pose2dDemacia(8.5, 3.66, Rotation2dDemacia.k180deg), 0.3, 0.4));
    points.add(new PathPoint(new Pose2dDemacia(8.26, 2.6, Rotation2dDemacia.fromDegrees(-115)), 0.3, 0.4));
    points.add(new PathPoint(new Pose2dDemacia(8.33, 0.7, Rotation2dDemacia.kCW_90deg), 1, 0.4));
    points.add(new PathPoint(new Pose2dDemacia(10, 0.7, Rotation2dDemacia.k180deg), 1, 0));
    points.add(new PathPoint(new Pose2dDemacia(13.7, 0.7, Rotation2dDemacia.kCW_90deg), 0, 0));

    FollowTrajectory trajectory = new FollowTrajectory(chassis, points);

    trajectory.addTrigger(new Pose2dDemacia(10.18, 1, Rotation2dDemacia.kZero), 0.2, 2 * Math.PI)
        .onTrue(RobotCommon.changeStateCommand(RobotStates.Trench));
    trajectory.addTrigger(new Pose2dDemacia(8.25, 2.5, Rotation2dDemacia.kCW_90deg), 0.5, 0.4 * Math.PI)
        .onTrue(RobotCommon.changeStateCommand(RobotStates.DriveWithIntake));
    trajectory.addTrigger(new Pose2dDemacia(13, 1, Rotation2dDemacia.kCW_90deg), 0.3, 2 * Math.PI)
        .onTrue(RobotCommon.changeStateCommand(RobotStates.Hub));

    return Commands.sequence(
        new InstantCommand(() -> {
          StateManager.getInstance().setStateChangeActivated(false);
          RobotCommon.setState(RobotStates.Trench);
          CommandScheduler.getInstance().schedule(new IntakeCommand(intake));
          CommandScheduler.getInstance().schedule(new ShinuaCommand(shinua));
          CommandScheduler.getInstance().schedule(new TurretCommand(turret));
          CommandScheduler.getInstance().schedule(new ShooterCommand(shooter));
          CommandScheduler.getInstance()
              .schedule(new WaitCommand(2).andThen(RobotCommon.changeStateCommand(RobotStates.Delivery)));
        }, chassis),
        trajectory);
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
              CommandScheduler.getInstance().schedule(new TurretCommand(turret));
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
    turret.setDefaultCommand(new TurretCommand(turret));

    /*
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
          if (turret.isAtMinLimit()) {
            mainLeds.startUserButton();
            turret.updatePositionByLimit();
          }
        }).ignoringDisable(true));
    buttons.addButton(ButtonsConstants.VOLTS_RANGE[1],
        new InstantCommand(() -> {
          mainLeds.startUserButton();
          shooter.resetHoodManual();
        }).ignoringDisable(true));
    buttons.addButton(ButtonsConstants.VOLTS_RANGE[2],
        new InstantCommand(() -> mainLeds.startUserButton()).andThen(RobotPose.getInstance().fullResetPoseCommand())
            .ignoringDisable(true));
    buttons.addButton(ButtonsConstants.VOLTS_RANGE[3],
        new Command() {
          private boolean isBrake = true;

          public void initialize() {
            mainLeds.startUserButton();
            isBrake = !isBrake;
            chassis.setNeutralMode(isBrake);
          };

          public boolean isFinished() {
            return true;
          };

        }.ignoringDisable(true));
    buttons.addButton(ButtonsConstants.VOLTS_RANGE[4],
        new Command() {
          private boolean isBrake = true;

          public void initialize() {
            mainLeds.startUserButton();
            isBrake = !isBrake;
            turret.setNeutralMode(isBrake);
          };

          public boolean isFinished() {
            return true;
          };
        }.ignoringDisable(true));

    driverController.upButton().onTrue(RobotCommon.changeStateCommand(RobotStates.DriveWithIntake));
    driverController.rightBumper().onTrue(
        new InstantCommand(() -> StateManager.getInstance().setStateChangeActivated(true)).ignoringDisable(true));
    driverController.leftBumper().onTrue(RobotCommon.changeStateCommand(RobotStates.Idle));
    driverController.rightButton().onTrue(new GetBallOutCommand(intake, shinua, driverController.rightButton()));
    driverController.leftButton().onTrue(new InstantCommand(DriveCommand::setPrecisionMode).ignoringDisable(true));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addBooleanProperty("is Red", RobotCommon::isRed, RobotCommon::setRed);
    builder.addBooleanProperty("is Comp", RobotCommon::isComp, RobotCommon::setComp);
    builder.addDoubleProperty("match Time", DriverStation::getMatchTime, null);
  }

  public void disableInit() {
    chassis.stop();
    intake.stopIntake();
    shinua.stop();
    turret.stop();
    shooter.stop();
    StateManager.getInstance().resetShift();
  }

  public void periodic() {
  }

  public static CommandController getDriverController() {
    return driverController;
  }

  public static PowerDistribution getPDH() {
    return PDH;
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

  public Command getAutonomousCommand() {
    // return null;
    return autoCommand;
  }

  public AutoFactory getAutoFactory() {
    return autoFactory;
  }
}