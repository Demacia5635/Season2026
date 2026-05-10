package frc.robot.intake.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.demacia.utils.log.LogManager;
import frc.demacia.utils.motors.SparkMaxMotor;
import frc.demacia.utils.motors.TalonFXMotor;

import frc.robot.intake.IntakeConstants;

public class ShinuaSubsystem extends SubsystemBase {
  private static ShinuaSubsystem instance;

  public static ShinuaSubsystem getInstance() {
    if (instance == null)
      instance = new ShinuaSubsystem();
    return instance;
  }

  private SparkMaxMotor motorIndexerClose;
  private SparkMaxMotor motorIndexerFar;
  private TalonFXMotor motorBattery;
  private TalonFXMotor motorIndexerOnTop;

  private ShinuaSubsystem() {
    motorIndexerClose = new SparkMaxMotor(IntakeConstants.INDEXER_CLOSE_CONFIG);
    motorIndexerFar = new SparkMaxMotor(IntakeConstants.INDEXER_FAR_CONFIG);
    motorIndexerOnTop = new TalonFXMotor(IntakeConstants.INDEXER_ON_TOP_CONFIG);
    motorBattery = new TalonFXMotor(IntakeConstants.BATTERY_CONFIG);

    setName("Shinua");
    SmartDashboard.putData("Shinua", this);
    SmartDashboard.putData("Shinua/Clear Sticky Fault", new InstantCommand(this::clearFaults).ignoringDisable(true));

    LogManager.log("Shinua Initalize");
  }

  public void clearFaults() {
    motorIndexerClose.clearFaults();
    motorIndexerFar.clearFaults();
  }

  public void checkElectronics() {
    motorIndexerClose.checkElectronics();
    motorIndexerFar.checkElectronics();
    motorIndexerOnTop.checkElectronics();
    motorBattery.checkElectronics();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Indexer Current", () -> motorIndexerOnTop.getCurrentCurrent(), null);
    builder.addDoubleProperty("Indexer Ve", () -> getIndexerOnTopVelocity(), null);
  }

  public double getIndexerOnTopCurrent() {
    return motorIndexerOnTop.getCurrentCurrent();
  }

  public double getIndexerOnTopVelocity() {
    return motorIndexerOnTop.getCurrentVelocity();
  }

  public void setNeutralMode(boolean isBrake) {
    motorBattery.setNeutralMode(isBrake);
    motorIndexerClose.setNeutralMode(isBrake);
    motorIndexerFar.setNeutralMode(isBrake);
    motorIndexerOnTop.setNeutralMode(isBrake);
  }

  public void setDutyIndexerClose(double pow) {
    motorIndexerClose.setDuty(pow);
  }

  public void stopIndexerClose() {
    motorIndexerClose.stop();
  }

  public void setDutyIndexerFar(double pow) {
    motorIndexerFar.setDuty(pow);
  }

  public void stopIndexerFar() {
    motorIndexerFar.stop();
  }

  public void setDutyIndexerOnTop(double pow) {
    motorIndexerOnTop.setDuty(pow);
  }

  public void stopIndexerOnTop() {
    motorIndexerOnTop.stop();
  }

  public void setEncoderPositionBattery(double position) {
    motorBattery.setEncoderPosition(position);
  }

  public void setPositionBattery(double position) {
    motorBattery.setPositionVoltage(position);
  }

  public void setPowerBattery(double power) {
    motorBattery.set(power);
  }

  public void stop() {
    motorBattery.stop();
    motorIndexerClose.stop();
    motorIndexerFar.stop();
    motorIndexerOnTop.stop();
  }
}