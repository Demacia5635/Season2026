package frc.robot.intake.subsystems;

import java.nio.Buffer;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.motors.SparkMaxMotor;
import frc.demacia.utils.motors.TalonFXMotor;
import frc.demacia.utils.motors.TalonSRXMotor;
import frc.robot.intake.IntakeConstants;

public class ShinuaSubsystem extends SubsystemBase {
  private SparkMaxMotor motorIndexerClose;
  private SparkMaxMotor motorIndexerFar;
  private TalonFXMotor motorBattery;
  private TalonFXMotor motorIndexerOnTop;
  private static ShinuaSubsystem instance;
  private boolean isMotorGoingToBeOnFireBatrry;
  private boolean isMotorGoingToBeOnFireCloseReadLine;
  private boolean isMotorGoingToBeOnFireFarReadLine;
  private boolean isMotorGoingToBeOnFireMotorOnTop;

  private ShinuaSubsystem() {
    motorIndexerClose = new SparkMaxMotor(IntakeConstants.INDEXER_CLOSE_CONFIG);
    motorIndexerFar = new SparkMaxMotor(IntakeConstants.INDEXER_FAR_CONFIG);
    motorIndexerOnTop = new TalonFXMotor(IntakeConstants.INDEXER_ON_TOP_CONFIG);
    motorBattery = new TalonFXMotor(IntakeConstants.BATTERY_CONFIG);
    setEncoderPositionBattery(IntakeConstants.MIN_POSITION);
    motorBattery.configPidFf(0);

    SmartDashboard.putData("Shinua/Top/set coast", new InstantCommand(() -> motorIndexerOnTop.setNeutralMode(false)).ignoringDisable(true));
    SmartDashboard.putData("Shinua/Top/set brake", new InstantCommand(() -> motorIndexerOnTop.setNeutralMode(true)).ignoringDisable(true));
    SmartDashboard.putData("Shinua/Left/set coast", new InstantCommand(() -> motorIndexerFar.setNeutralMode(false)).ignoringDisable(true));
    SmartDashboard.putData("Shinua/Left/set brake", new InstantCommand(() -> motorIndexerFar.setNeutralMode(true)).ignoringDisable(true));
    SmartDashboard.putData("Shinua/Right/set coast", new InstantCommand(() -> motorIndexerClose.setNeutralMode(false)).ignoringDisable(true));
    SmartDashboard.putData("Shinua/Right/set brake", new InstantCommand(() -> motorIndexerClose.setNeutralMode(true)).ignoringDisable(true));
    SmartDashboard.putData("Shinua/Battery/set coast", new InstantCommand(() -> motorBattery.setNeutralMode(false)).ignoringDisable(true));
    SmartDashboard.putData("Shinua/Battery/set brake", new InstantCommand(() -> motorBattery.setNeutralMode(true)).ignoringDisable(true));
  }

  public static ShinuaSubsystem getInstance() {
    if (instance == null)
      instance = new ShinuaSubsystem();
    return instance;
  }

  public void checkElectronics() {
    motorIndexerClose.checkElectronics();
    motorIndexerFar.checkElectronics();
    motorIndexerOnTop.checkElectronics();
    motorBattery.checkElectronics();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
      builder.addBooleanProperty("battery motor lock", this::getIsBattryLock, this::setIndexrBatrryLock);
      builder.addBooleanProperty("motor on top lock", this::getIsIndexrOnTopLock, this::setIndexrOnTop);
      builder.addBooleanProperty("motor close lock", this::getIsIndexrCloseLock, this::setIndexrCloseLock);
      builder.addBooleanProperty("motor far lock", this::getIsIndexerFarLock, this::setIndexrFarLock);
  }

  @Override
  public void periodic(){
    if(motorIndexerOnTop.getCurrentCurrent() > 25) isMotorGoingToBeOnFireMotorOnTop =true;
    if(motorBattery.getCurrentCurrent() > 25) isMotorGoingToBeOnFireBatrry = true;
    if(motorIndexerClose.getCurrentCurrent() > 25) isMotorGoingToBeOnFireCloseReadLine = true; 
    if(motorIndexerFar.getCurrentCurrent() > 25) isMotorGoingToBeOnFireFarReadLine = true;
  }


  private boolean getIsIndexrOnTopLock(){
    return isMotorGoingToBeOnFireBatrry;
  }

  private boolean getIsIndexrCloseLock(){
    return isMotorGoingToBeOnFireCloseReadLine;
  }

  private boolean getIsIndexerFarLock(){
    return isMotorGoingToBeOnFireFarReadLine;
  }

  private boolean getIsBattryLock(){
    return isMotorGoingToBeOnFireBatrry;
  }

  private void setIndexrOnTop(boolean lock){
    this.isMotorGoingToBeOnFireMotorOnTop = lock;
  }

  private void setIndexrCloseLock(boolean lock){
    this.isMotorGoingToBeOnFireCloseReadLine = lock;
  }

  private void setIndexrFarLock(boolean lock){
    this.isMotorGoingToBeOnFireFarReadLine = lock;
  }

  private void setIndexrBatrryLock(boolean lock){
    this.isMotorGoingToBeOnFireBatrry = lock;
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

  public boolean isAtMin() {
    return IntakeConstants.MIN_POSITION >= motorBattery.getCurrentPosition();
  }

  public boolean isAtMax() {
    return IntakeConstants.MAX_POSITION <= motorBattery.getCurrentPosition();
  }

  public void stop() {
    motorBattery.stop();
    motorIndexerClose.stop();
    motorIndexerFar.stop();
    motorIndexerOnTop.stop();
  }
}