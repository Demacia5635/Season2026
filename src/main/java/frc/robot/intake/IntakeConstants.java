// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import frc.demacia.utils.motors.BaseMotorConfig.Canbus;
import frc.demacia.utils.motors.SparkMaxConfig;
import frc.demacia.utils.motors.TalonFXConfig;

public class IntakeConstants {
        public static final Canbus INTAKE_CANBUS = Canbus.Rio;

        public static final int INTAKE_ID = 20;
        public static final String INTAKE_NAME = "Intake/Motor";

        public static final TalonFXConfig INTAKE_CONFIG = new TalonFXConfig(INTAKE_ID, INTAKE_CANBUS, INTAKE_NAME)
                        .withInvert(true)
                        .withRadiansMotor(12)
                        .withCurrent(30);

        public static final int INDEXER_ON_TOP_ID = 30;
        public static final String INDEXER_TOP_NAME = "Shinua/Top";
        public static final TalonFXConfig INDEXER_ON_TOP_CONFIG = new TalonFXConfig(INDEXER_ON_TOP_ID, Canbus.CANIvore,
                        INDEXER_TOP_NAME)
                        .withRadiansMotor(4)
                        .withBrake(true)
                        .withVolts(8)
                        .withCurrent(30)
                        .withInvert(true);

        public static final int INDEXER_CLOSE_ID = 31;
        public static final String INDEXER_CLOSE_NAME = "Indexer Close motor";
        public static final SparkMaxConfig INDEXER_CLOSE_CONFIG = new SparkMaxConfig(INDEXER_CLOSE_ID,
                        INDEXER_CLOSE_NAME)
                        .withBrake(true).withInvert(true).withCurrent(15).withVolts(8);

        public static final int INDEXER_FAR_ID = 32;
        public static final String INDEXER_FAR_NAME = "Indexer far motor";
        public static final SparkMaxConfig INDEXER_FAR_CONFIG = new SparkMaxConfig(INDEXER_FAR_ID, INDEXER_FAR_NAME)
                        .withBrake(true)
                        .withCurrent(15)
                        .withVolts(8)
                        .withInvert(false);

        public static final int BATTERY_ID = 33;
        public static final String BATTER_NAME = "Shinua/Battery";
        public static final TalonFXConfig BATTERY_CONFIG = new TalonFXConfig(BATTERY_ID, Canbus.CANIvore, BATTER_NAME)
                        .withBrake(true)
                        .withInvert(false)
                        .withCurrent(15)
                        .withVolts(10);

}