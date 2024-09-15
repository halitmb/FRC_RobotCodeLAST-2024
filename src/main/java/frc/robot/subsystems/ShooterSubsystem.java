// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

// import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;


public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkMax m_upperShooterMotor = new CANSparkMax(ShooterConstants.kUpperShooterMotorId, MotorType.kBrushless);
  private final CANSparkMax m_lowerShooterMotor = new CANSparkMax(ShooterConstants.kLowerShooterMotorId, MotorType.kBrushless);

  public ShooterSubsystem() {
    m_upperShooterMotor.setInverted(false);
    m_lowerShooterMotor.setInverted(false);
  }

  public void runShooter() {
    double newVoltage = ShooterConstants.kShooterVoltage;
    m_upperShooterMotor.setVoltage(newVoltage);
    m_lowerShooterMotor.setVoltage(newVoltage);
  }
  
  public void stopShooter() {
    double newVoltage = 0;
    m_upperShooterMotor.set(newVoltage);
    m_lowerShooterMotor.set(newVoltage);
  }

  public Command runShooterCommand() { return run(() -> runShooter()); }
  public Command runShooterOnceCommand() { return runOnce(() -> runShooter()); }
  public Command stopShooterCommand() { return run(() -> stopShooter()); }
  public Command stopShooterOnceCommand() { return runOnce(() -> stopShooter()); }
}
