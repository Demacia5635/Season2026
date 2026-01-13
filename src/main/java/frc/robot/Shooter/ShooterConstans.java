// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter;

import frc.demacia.utils.motors.TalonFXConfig;
import frc.demacia.utils.motors.BaseMotorConfig.Canbus;

/** Add your docs here. */

public class ShooterConstans {

    public static final int shooterMotorID = 12;
    public static final Canbus shooterMotorCanbus = Canbus.Rio;
    public static final String shooterMotorName = "Shooter Motor";

    public static final TalonFXConfig SHOOTER_MOTOR_CONFIG = new TalonFXConfig(shooterMotorID, shooterMotorCanbus, shooterMotorName)
    .withFeedForward(0, 0)
    .withPID(0, 0, 0, 0, 0, 0, 0)
    .withMeterMotor(0, 0);
}
