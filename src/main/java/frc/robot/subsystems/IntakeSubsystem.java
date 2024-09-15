// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstans;

import edu.wpi.first.wpilibj.DigitalInput;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;


public class IntakeSubsystem extends SubsystemBase {
  private final WPI_VictorSPX m_intakeMotor = new WPI_VictorSPX(IntakeConstans.kIntakeMotorId);
  private final WPI_VictorSPX m_feederMotor = new WPI_VictorSPX(IntakeConstans.kFeederMotorId);
  private DigitalInput m_objectSensor = new DigitalInput(IntakeConstans.kObjectSensorId);

  private double m_lastIntakeVoltage = 0;
  private double m_lastFeederVoltage = 0;

  public IntakeSubsystem() {
    m_intakeMotor.setInverted(true);
    m_feederMotor.setInverted(true);
  }

  public boolean getSensorReading() {
    return m_objectSensor.get();
  }

  public void runFeederIn() {
    m_lastFeederVoltage = IntakeConstans.kFeederVoltage;
    m_feederMotor.setVoltage(m_lastFeederVoltage);
  }

  public void runFeederOut() {
    m_lastFeederVoltage = -IntakeConstans.kFeederVoltage;
    m_feederMotor.setVoltage(m_lastFeederVoltage);
  }

  public void stopFeeder() {
    m_lastFeederVoltage = 0;
    m_feederMotor.setVoltage(m_lastFeederVoltage);
  }

  public void runIntakeIn() {
    m_lastIntakeVoltage = IntakeConstans.kIntakeVoltage;
    m_intakeMotor.setVoltage(m_lastIntakeVoltage);
  }

  public void runIntakeOut() {
    m_lastIntakeVoltage = -IntakeConstans.kIntakeVoltage/2;
    m_intakeMotor.setVoltage(m_lastIntakeVoltage);
  }

  public void stopIntake() {
    m_lastIntakeVoltage = 0;
    m_intakeMotor.setVoltage(m_lastIntakeVoltage);
  }

  public Command runFeederInCommand()       { return run(() -> runFeederIn()); }
  public Command runFeederInOnceCommand()   { return runOnce(() -> runFeederIn()); }
  public Command runFeederOutCommand()      { return run(() -> runFeederOut()); }
  public Command runFeederOutOnceCommand()  { return runOnce(() -> runFeederOut()); }
  public Command stopFeederCommand()        { return run(() -> stopFeeder()); }
  public Command stopFeederOnceCommand()    { return runOnce(() -> stopFeeder()); }
  public Command runIntakeInCommand()       { return run(() -> runIntakeOut()); }
  public Command runIntakeInOnceCommand()   { return runOnce(() -> runIntakeIn()); }
  public Command runIntakeOutCommand()      { return run(() -> runIntakeOut()); }
  public Command runIntakeOutOnceCommand()  { return runOnce( () -> runIntakeOut()); }
  public Command stopIntakeCommand()        { return run(() -> stopIntake()); }
  public Command stopIntakeOnceCommand()    { return run( () -> stopIntake()); }

  @Override
  public void periodic() {
    m_intakeMotor.setVoltage(m_lastIntakeVoltage);
    m_feederMotor.setVoltage(m_lastFeederVoltage);
  }
}
