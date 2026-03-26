// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demacia.path.Trgectory;

public class TrajectoryConstants {
    public static final double FIELD_LENGTH = -1; // X in field based
    public static final double FIELD_HEIGHT = -1; // y in field based

    public static final double MAX_POSITION_THRESHOLD = 0.03; // in meters;
    public static final double MAX_ROTATION_THRESHOLD = Math.toRadians(2);
    
   


    public class PathsConstraints {
        public static final double MAX_VELOCITY = 2.5;
        public static final double MAX_ACCEL = 6;

    }

}