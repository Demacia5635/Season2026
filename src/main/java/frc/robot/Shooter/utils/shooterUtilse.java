// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter.utils;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.geometry.Rotation2d;
import frc.demacia.utils.geometry.Translation2d;
import frc.robot.Shooter.commands.ShooterFollowCommand;
import frc.robot.Shooter.subsystem.Shooter;

public class shooterUtilse extends SubsystemBase {
  /** Creates a new shooterUtilse. */

  Shooter shooter;

  Translation3d shooterVelInVector;
  private double shooterAngleFucer;
  Translation3d shooterVelInFucerVector;

  private double turret;

  private double ShooterVelocity;
  private double shooterAngle;
  private double shooterVelInFucer;


  public shooterUtilse() {
    this.shooterVelInFucer = ShooterFollowCommand.VelocityInFucer;
    this.ShooterVelocity = shooter.getShooterVelocity();
  }


  public Translation3d getCurentShooterVelInVector(){
    shooterVelInVector = new Translation3d(ShooterVelocity, 
      new Rotation3d(0, shooterAngle, turret)
    );

    return shooterVelInVector;
  }

  public Translation3d getFucerShooterVelInVector(){
    shooterVelInFucerVector = new Translation3d(shooterVelInFucer, new Rotation3d(0, shooterAngleFucer, turret));

    return shooterVelInFucerVector;
  }

  

  @Override
  public void periodic() { 
    // This method will be called once per scheduler run
  } 
}
