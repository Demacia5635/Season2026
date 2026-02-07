// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.demacia.utils.controller.CommandController;
import frc.demacia.utils.controller.CommandController.ControllerType;
import frc.demacia.utils.motors.TalonFXConfig;
import frc.demacia.utils.motors.TalonFXMotor;
import frc.demacia.utils.motors.TalonSRXConfig;
import frc.demacia.utils.motors.TalonSRXMotor;
import frc.demacia.utils.motors.BaseMotorConfig.Canbus;
import frc.demacia.utils.sensors.LimitSwitch;
import frc.demacia.utils.sensors.LimitSwitchConfig;
import frc.robot.intake.command.BatteryLid;
import frc.robot.intake.command.CloseBattery;
import frc.robot.intake.command.ShootBattery;

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

  // public static Chassis chassis;
  // CommandController driverController = new CommandController(0,
  // ControllerType.kPS5);

  // The robot's subsystems and commands are defined here...

  // Replace with CommandPS4Controller or CommandJoystick if needed

  final TalonSRXMotor redlineLeft;
  final TalonSRXMotor redlineRight;
  final TalonFXMotor krakenMotor;
  
  private BatteryLid batterySubsystem;

  static double krakenPow = 0d;
  static double leftPow = 0d;
  static double rightPow = 0d;

  LimitSwitch sensor1 = new LimitSwitch(new LimitSwitchConfig(1, "magnetic switch 1").withInvert(true));
  LimitSwitch sensor2 = new LimitSwitch(new LimitSwitchConfig(2, "magnetic switch 2").withInvert(true));
  LimitSwitch sensor3 = new LimitSwitch(new LimitSwitchConfig(3, "magnetic switch 3").withInvert(true));

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    SmartDashboard.putData("RC", this);

    krakenMotor = new TalonFXMotor(new TalonFXConfig(21, Canbus.Rio, "Feeder").withBrake(true).withInvert(true));
    redlineLeft = new TalonSRXMotor(new TalonSRXConfig(22, "RedLine Left motor").withBrake(true));
    redlineRight = new TalonSRXMotor(new TalonSRXConfig(23, "Red Line Right Motor").withBrake(true).withInvert(true));
   
    batterySubsystem = new BatteryLid();
    Command shinuaCommand = new FunctionalCommand(
        () -> {
        },
        () -> {
          krakenMotor.setDuty(krakenPow);
          redlineLeft.setDuty(leftPow);
          redlineRight.setDuty(rightPow);
        },
        (isInterrupted) -> {
          krakenMotor.stop();
          redlineLeft.stop();
          redlineRight.stop();
        },
        () -> false);
    SmartDashboard.putData("Shinua Command", shinuaCommand);
    SmartDashboard.putData("Reset Motor position min", new InstantCommand(() -> {
      batterySubsystem.setEncoderPosition(0); 
    }).ignoringDisable(true));
    

    SmartDashboard.putData("Shoot Battery", new ShootBattery(batterySubsystem));
    SmartDashboard.putData("Close Lid", new CloseBattery(batterySubsystem));
    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());

    // chassis = new Chassis(MK4iChassisConstants.CHASSIS_CONFIG);

    // Configure the trigger bindings
    // addStatesToElasticForTesting();
    configureBindings();
  }

  public void addStatesToElasticForTesting() {
    SendableChooser<RobotCommon.robotStates> robotStateChooser = new SendableChooser<>();
    for (RobotCommon.robotStates state : RobotCommon.robotStates.class.getEnumConstants()) {
      robotStateChooser.addOption(state.name(), state);
    }
    robotStateChooser.onChange(state -> RobotCommon.currentState = state);
    SmartDashboard.putData("Robot State Chooser", robotStateChooser);

    SendableChooser<RobotCommon.Shifts> shiftsChooser = new SendableChooser<>();
    for (RobotCommon.Shifts state : RobotCommon.Shifts.class.getEnumConstants()) {
      shiftsChooser.addOption(state.name(), state);
    }
    shiftsChooser.onChange(state -> RobotCommon.currentShift = state);
    SmartDashboard.putData("Shifts Chooser", shiftsChooser);
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
  public static boolean isShooting = false;

  private void configureBindings() {
    // DriveCommand driveCommand = new DriveCommand(chassis, driverController);
    // chassis.setDefaultCommand(driveCommand);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("kraken pow", RobotContainer::getKrakenPow, RobotContainer::setKrakenPow);
    builder.addDoubleProperty("left pow", RobotContainer::getLeftPow, RobotContainer::setLeftPow);
    builder.addDoubleProperty("right pow", RobotContainer::getRightPow, RobotContainer::setRightPow);

    builder.addBooleanProperty("is comp", () -> RobotCommon.isComp, (isComp) -> RobotCommon.isComp = isComp);
    builder.addBooleanProperty("is red", () -> RobotCommon.isRed, (isRed) -> RobotCommon.isRed = isRed);

    builder.addBooleanProperty("change is Robot Calibrated for testing", () -> RobotCommon.isRobotCalibrated,
        (isRobotCalibrated) -> RobotCommon.isRobotCalibrated = isRobotCalibrated);
    builder.addDoubleProperty("change Accuracy for testing", () -> RobotCommon.targetAccuracy,
        (targetAccuracy) -> RobotCommon.targetAccuracy = targetAccuracy);
  }

  static public void updateCommon() {
    Translation2d currentPoseFromHub = RobotCommon.currentRobotPose.getTranslation().minus(HUB_POS);
    RobotCommon.currentDistanceFromTarget = currentPoseFromHub.getNorm();
    RobotCommon.currentAngleFormTarget = currentPoseFromHub.getAngle().getRadians();
    RobotCommon.currentWantedTurretAngle = RobotCommon.currentWantedTurretAngle
        - RobotCommon.currentRobotPose.getRotation().getRadians();

    Translation2d futurePoseFromHub = RobotCommon.futureRobotPose.getTranslation().minus(HUB_POS);
    RobotCommon.futureDistanceFromTarget = futurePoseFromHub.getNorm();
    RobotCommon.futureAngleFormTarget = futurePoseFromHub.getAngle().getRadians();
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
    return null;
  }

  public TalonSRXMotor getRedlineLeft() {
    return redlineLeft;
  }

  public TalonSRXMotor getRedlineRight() {
    return redlineRight;
  }

  public TalonFXMotor getKrakenMotor() {
    return krakenMotor;
  }

  public static double getKrakenPow() {
    return krakenPow;
  }

  public static void setKrakenPow(double krakenPow) {
    RobotContainer.krakenPow = krakenPow;
  }

  public static double getLeftPow() {
    return leftPow;
  }

  public static void setLeftPow(double leftPow) {
    RobotContainer.leftPow = leftPow;
  }

  public static double getRightPow() {
    return rightPow;
  }

  public static void setRightPow(double rightPow) {
    RobotContainer.rightPow = rightPow;
  }

  public static boolean isShooting() {
    return isShooting;
  }

  public static void setShooting(boolean isShooting) {
    RobotContainer.isShooting = isShooting;
  }

}