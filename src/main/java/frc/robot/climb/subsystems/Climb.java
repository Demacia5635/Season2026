// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.climb.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
<<<<<<< HEAD

public class Climb extends SubsystemBase {
  /** Creates a new Climb. */
  public Climb() {}

=======
import frc.demacia.utils.motors.TalonFXMotor;
import frc.demacia.utils.sensors.LimitSwitch;
import frc.robot.climb.constants.ClimbConstants;

public class Climb extends SubsystemBase {
  private TalonFXMotor motor1;
  private LimitSwitch limitSwitch;

  /** Creates a new Climb. */
  public Climb() {
    motor1 = new TalonFXMotor(ClimbConstants.MOTOR_CONFIG);
    limitSwitch = new LimitSwitch(ClimbConstants.lIMIT_Config);
  }

  public void setDuty(double power) {
    motor1.setDuty(power);
  }

  public void stop() {
    motor1.stop();
  }

  public boolean getLimit(){
    return limitSwitch.get();
  }
>>>>>>> 26a2108cdc4622c206a120566ef7ef0e6ec5f8f7
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
