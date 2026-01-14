// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter;

import frc.demacia.utils.motors.TalonFXConfig;
import frc.demacia.utils.motors.BaseMotorConfig.Canbus;

/** Add your docs here. */

public class ShooterConstans {

    public static final int shooterMotorID = 21;
    public static final Canbus shooterMotorCanbus = Canbus.Rio;
    public static final String shooterMotorName = "Shooter Motor";

    public static final TalonFXConfig SHOOTER_MOTOR_CONFIG = new TalonFXConfig(shooterMotorID, shooterMotorCanbus, shooterMotorName)
    .withFeedForward(0.0045, 0)
    .withInvert(true)
    .withRampTime(0.5)
    .withBrake(false)
    .withPID(2.2, 0, 0, 0.21073, 0.245, 0.05, 0)
    .withMeterMotor(1, 4 * 0.0254);

    public static final int INDEXER_ID = 60;
    public static final Canbus CANBUS_MOVE_TO_SOTER_MOTOR = Canbus.Rio;
    public static final String INDEXER_NAME = "Indexer Motor";


    public static final TalonFXConfig INDEXER_CONFIG = new TalonFXConfig(INDEXER_ID, CANBUS_MOVE_TO_SOTER_MOTOR, INDEXER_NAME)
    .withInvert(true);
}
