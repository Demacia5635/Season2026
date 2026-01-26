// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
<<<<<<< HEAD
=======
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.demacia.utils.DemaciaUtils;
>>>>>>> origin/Shooter
import frc.demacia.utils.chassis.Chassis;
import frc.demacia.utils.controller.CommandController;
import frc.demacia.utils.controller.CommandController.ControllerType;
import frc.demacia.utils.log.LogManager;
import frc.robot.Shooter.commands.HoodCalibrationCommand;
<<<<<<< HEAD
=======
import frc.robot.Shooter.commands.ShootOnTheFly;
import frc.robot.Shooter.commands.ShooterCommand;
>>>>>>> origin/Shooter
import frc.robot.Shooter.subsystem.Shooter;
import frc.robot.chassis.MK4iChassisConstants;
import frc.robot.chassis.commands.ResetModule;

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

<<<<<<< HEAD
  public static RobotContainer intsnace;

  boolean isComp = false;
  boolean isRed = false;
  public Chassis chassis;
  public CommandController driverController = new CommandController(Constants.OperatorConstants.kDriverControllerPort, ControllerType.kPS5);
  public Shooter shooter;
  private boolean hasRemovedFromLog = false;
  

=======
  public static boolean isComp = false;
  private static boolean hasRemovedFromLog = false;
  public static boolean isRed = false;
  Field2d field2d;
  Field2d questField2d;
  Chassis chassis;
  CommandController driverController = new CommandController(0, ControllerType.kPS5);
  Shooter shooter;

  // The robot's subsystems and commands are defined here...

  // Replace with CommandPS4Controller or CommandJoystick if needed

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
>>>>>>> origin/Shooter
  public RobotContainer() {
    intsnace = this;
    chassis = new Chassis(MK4iChassisConstants.CHASSIS_CONFIG);
    shooter = new Shooter(chassis);

<<<<<<< HEAD
    SmartDashboard.putData("Robot Container", this);
    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
    
    SmartDashboard.putData("Reset Module Back Left", new ResetModule(chassis, 2, 0).ignoringDisable(true));
    SmartDashboard.putData("Hood calibration", new HoodCalibrationCommand(shooter));

=======
    SmartDashboard.putData("chassis/Reset Module Back Left", new ResetModule(chassis, 2, 0).ignoringDisable(true));
    SmartDashboard.putData("shooter/Hood calibration", new HoodCalibrationCommand(shooter));
    // Configure the trigger bindings
    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
>>>>>>> origin/Shooter
    configureBindings();

  }

  /**
<<<<<<< HEAD
=======
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
>>>>>>> origin/Shooter
   */
  public static boolean isShooting = false;

  private void configureBindings() {
<<<<<<< HEAD
    
    driverController.upButton().onTrue(new InstantCommand(()->chassis.headToTargetToggle()));
    driverController.downButton().onTrue(new InstantCommand(()->shooter.setIndexerPower(1)));
    driverController.leftBumper().onTrue(new InstantCommand(()->shooter.setIndexerPower(0)));
=======
    DriveCommand driveCommand = new DriveCommand(chassis, driverController);
    chassis.setDefaultCommand(driveCommand);
    //shooter.setDefaultCommand(new ShootOnTheFly(chassis, shooter));
    shooter.setDefaultCommand(new ShooterCommand(shooter, chassis));
    driverController.rightButton().onTrue(new RunCommand(() -> {}, shooter));
    //   RobotContainer.isShooting = !RobotContainer.isShooting;
    //   if (isShooting) {
    //     CommandScheduler.getInstance().schedule(new InstantCommand(() -> new ShooterCommand(shooter, chassis).schedule()));
    //   }
    //   else {
    //     CommandScheduler.getInstance().schedule(new InstantCommand(() -> {}, shooter));
    //   }
    // }));
    driverController.upButton().onTrue(new InstantCommand(() -> driveCommand.setActiveToHub()));
    driverController.downButton().onTrue(new InstantCommand(() -> shooter.setIndexerPower(1)));
    driverController.leftBumper().onTrue(new InstantCommand(() -> shooter.setIndexerPower(0)));
>>>>>>> origin/Shooter
  }

  public boolean isRed() {
    return isRed;
  }

  public void isRed(boolean isRed) {
    this.isRed = isRed;
  }

  public boolean isComp() {
    return isComp;
  }

<<<<<<< HEAD
  public void isComp(boolean isComp) {
    this.isComp = isComp;
    if(!hasRemovedFromLog && isComp) {
=======
  public static void setIsComp(boolean isComp) {
    RobotContainer.isComp = isComp;
    if (!hasRemovedFromLog && isComp) {
>>>>>>> origin/Shooter
      hasRemovedFromLog = true;
      LogManager.removeInComp();
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
<<<<<<< HEAD
    builder.addBooleanProperty("isRed", this::isRed, this::isRed);
    builder.addBooleanProperty("isComp", this::isComp, this::isComp);
=======
    builder.addBooleanProperty("isRed", RobotContainer::getIsRed, RobotContainer::setIsRed);
    builder.addBooleanProperty("isComp", RobotContainer::getIsComp, RobotContainer::setIsComp);
    // builder.addDoubleProperty("Angle", () -> shooter.angle, (newAngle) ->
    // shooter.angle = newAngle);
>>>>>>> origin/Shooter
  }

  /**
   */
  public Command getAutonomousCommand() {
    return null;
  }
}