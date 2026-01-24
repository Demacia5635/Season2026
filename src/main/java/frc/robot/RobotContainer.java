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
import frc.demacia.utils.chassis.Chassis;
import frc.demacia.utils.controller.CommandController;
import frc.demacia.utils.controller.CommandController.ControllerType;
import frc.demacia.utils.log.LogManager;
import frc.robot.Shooter.commands.HoodCalibrationCommand;
import frc.robot.Shooter.subsystem.Shooter;
import frc.robot.chassis.MK4iChassisConstants;
import frc.robot.chassis.commands.ResetModule;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer implements Sendable{

  public static RobotContainer intsnace;

  boolean isComp = false;
  boolean isRed = false;
  public Chassis chassis;
  public CommandController driverController = new CommandController(Constants.OperatorConstants.kDriverControllerPort, ControllerType.kPS5);
  public Shooter shooter;
  private boolean hasRemovedFromLog = false;
  

  public RobotContainer() {
    intsnace = this;
    chassis = new Chassis(MK4iChassisConstants.CHASSIS_CONFIG);
    shooter = new Shooter(chassis);

    SmartDashboard.putData("Robot Container", this);
    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
    
    SmartDashboard.putData("Reset Module Back Left", new ResetModule(chassis, 2, 0).ignoringDisable(true));
    SmartDashboard.putData("Hood calibration", new HoodCalibrationCommand(shooter));

    configureBindings();

  }

  /**
   */
  private void configureBindings() {
    
    driverController.upButton().onTrue(new InstantCommand(()->chassis.headToTargetToggle()));
    driverController.downButton().onTrue(new InstantCommand(()->shooter.setIndexerPower(1)));
    driverController.leftBumper().onTrue(new InstantCommand(()->shooter.setIndexerPower(0)));
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

  public void isComp(boolean isComp) {
    this.isComp = isComp;
    if(!hasRemovedFromLog && isComp) {
      hasRemovedFromLog = true;
      LogManager.removeInComp();
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addBooleanProperty("isRed", this::isRed, this::isRed);
    builder.addBooleanProperty("isComp", this::isComp, this::isComp);
  }

  /**
   */
  public Command getAutonomousCommand() {
    return null;
  }
}