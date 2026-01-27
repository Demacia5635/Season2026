// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.utils.chassis.Chassis;
import frc.demacia.utils.controller.CommandController;
import frc.robot.Shooter.ShooterConstans;
import frc.robot.Shooter.subsystem.Shooter;
import frc.demacia.utils.controller.CommandController;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShooterCommand extends Command {
  /** Creates a new shooterCommand. */

  Chassis chassis;
  Shooter shooter;
  double vel = 0;
  double hoodAngle = 0;
  CommandController controller;
  double[] shooterValues = new double[2];

  public ShooterCommand(Shooter shooter, Chassis chassis, CommandController controller) {
    this.shooter = shooter;
    this.chassis = chassis;
    this.controller = controller;
    addRequirements(shooter);
    SmartDashboard.putData(this);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Flywheel vel", () -> vel, (x) -> vel = x);
    builder.addDoubleProperty("Hood Angle", () -> hoodAngle, (x) -> hoodAngle = x);
    builder.addDoubleArrayProperty("Wanted", () -> shooterValues, null);
    builder.addDoubleProperty("dis", () -> hubToChassis.getNorm(), null);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  Translation2d hubToChassis = Translation2d.kZero;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d predictedPose = chassis.getPoseWithVelocity();
    // Pose2d predictedPose = chassis.getPose();
    hubToChassis = ShooterConstans.HUB_POSE_Translation3d.toTranslation2d()
        .minus(predictedPose.getTranslation());
    shooterValues = ShooterConstans.SHOOTER_LOOKUP_TABLE.get(hubToChassis.getNorm());
    // frc.demacia.utils.log.LogManager.log("NORM: " + hubToChassis.getNorm());
    
    
    shooter.setHoodAngle(Math.toRadians(hoodAngle));

    // shooter.setHoodAngle(shooterValues[1]);
    // shooter.setFlywheelVel(shooterValues[0]);

    //shooter.setHoodAngle(Math.toRadians(hoodAngle));
    //shooter.setFlywheelVel(vel);



    // shooter.setHoodAngle(Math.toRadians(hoodAngle));
    // shooter.setFlywheelPower(vel);
    //0.18863

    // shooter.setHoodPower(controller.getLeftY() * 0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
