// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
// import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
// import edu.wpi.first.units.Angle;
// import edu.wpi.first.units.MutableMeasure;
// import edu.wpi.first.units.Units;
// import edu.wpi.first.units.Velocity;
// import edu.wpi.first.units.Voltage;


public class ArmSubsystem extends SubsystemBase {
  public DigitalInput m_armUpperLimit = new DigitalInput(ArmConstants.kArmUpperSensorId);
  public DigitalInput m_armLowerLimit = new DigitalInput(ArmConstants.kArmLowerSensorId);
  private ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

  private WPI_VictorSPX m_armMotor1 = new WPI_VictorSPX(ArmConstants.kArmMotor1Id);
  private WPI_VictorSPX m_armMotor2 = new WPI_VictorSPX(ArmConstants.kArmMotor2Id);
  private WPI_VictorSPX m_armMotor3 = new WPI_VictorSPX(ArmConstants.kArmMotor3Id);
  private WPI_VictorSPX m_armMotor4 = new WPI_VictorSPX(ArmConstants.kArmMotor4Id);
  public double m_armAngleSetpoint = 0;
  public double m_lastVoltage = 0;
  public boolean isPIDActive = false;

  // private MutableMeasure<Voltage> m_voltage1 = MutableMeasure.mutable(Units.Volts.of(0));
  // private MutableMeasure<Voltage> m_voltage2 = MutableMeasure.mutable(Units.Volts.of(0));
  // private MutableMeasure<Voltage> m_voltage3 = MutableMeasure.mutable(Units.Volts.of(0));
  // private MutableMeasure<Voltage> m_voltage4 = MutableMeasure.mutable(Units.Volts.of(0));
 
  // private MutableMeasure<Velocity<Angle>> m_velocity = MutableMeasure.mutable(Units.DegreesPerSecond.of(0));
 
  // private MutableMeasure<Angle> m_position = MutableMeasure.mutable(Units.Degrees.of(0));

  // private SysIdRoutine sysIdRoutine = new SysIdRoutine(
  //   new SysIdRoutine.Config(ArmConstants.kSysIdRampRate, ArmConstants.kSysIdStepVoltage, ArmConstants.kSysIdTimeout), 
  //   new SysIdRoutine.Mechanism(
  //     voltage -> {
  //       m_armMotor1.setVoltage(voltage.baseUnitMagnitude());
  //       m_armMotor2.setVoltage(voltage.baseUnitMagnitude());
  //       m_armMotor3.setVoltage(voltage.baseUnitMagnitude());
  //       m_armMotor4.setVoltage(voltage.baseUnitMagnitude());
  //     }, 
  //     log -> {
  //               // Record a frame for the arm motor.
  //               log.motor("arm")
  //                   .voltage(
  //                       m_voltage1.mut_replace(
  //                           m_armMotor1.getMotorOutputVoltage(), Units.Volts))
  //                   // .voltage(
  //                   //     m_voltage2.mut_replace(
  //                   //         m_armMotor2.getMotorOutputVoltage(), Units.Volts))
  //                   //         .voltage(
  //                   //     m_voltage3.mut_replace(
  //                   //         m_armMotor3.getMotorOutputVoltage(), Units.Volts))
  //                   // .voltage(
  //                   //     m_voltage4.mut_replace(
  //                   //         m_armMotor4.getMotorOutputVoltage(), Units.Volts))
  //                   .angularPosition(m_position.mut_replace(m_gyro.getRate(), Units.Degrees))
  //                   .angularVelocity(
  //                       m_velocity.mut_replace(m_gyro.getRate(), Units.DegreesPerSecond));
  //     }, 
  //     this));
  
  private ArmFeedforward m_armFeedforward = new ArmFeedforward(
    ArmConstants.kArmKs,
    ArmConstants.kArmKg,
    ArmConstants.kArmKv
  );

  private PIDController m_armPIDController = new PIDController(
    ArmConstants.kArmKp, 
    ArmConstants.kArmKi, 
    ArmConstants.kArmKd
  );


  public ArmSubsystem() {
    // m_armMotor1.setInverted(false);
    // m_armMotor2.setInverted(false);
    // m_armMotor3.setInverted(false);
    // m_armMotor4.setInverted(false);
  }

  
  public double calculateArmAngle(double distance) {
    // Takes distance in cm, returns angle in degrees

    // Distance 0dan küçük gelirse apriltag görülmemiştir
    if (distance < 0) { return distance; }

    // Bizim ölçülerimize göre 163cmden kısa 
    // uzaklıkta 0 derecede atıyoruz
    if (distance <= 163) { return 0; }

    double calculatedAngle = 26.519 * Math.log(1.87483*distance + 49.8617) - 155.862;
    if (calculatedAngle < 0) { return 0; }
    return calculatedAngle;
  }
  
  public void increaseArmAngle(double step) {
    m_armAngleSetpoint += step;  
    setArmAngle();
  }
   
  public Command increaseArmAngleCommand(DoubleSupplier step) {
    return runOnce(() -> increaseArmAngle(step.getAsDouble()));
  }
 
  public void decreaseArmAngle(double step) {
    m_armAngleSetpoint -= step;  
    setArmAngle();
  }
  
  public Command decreaseArmAngleCommand(DoubleSupplier step) {
    return runOnce(() -> decreaseArmAngle(step.getAsDouble()));
  }
 
  public void setArmAngle(double angleSetpoint) {
    // sıfırdan küçük, doksandan büyük değer verilemez
    if (angleSetpoint < 0 && 90 < angleSetpoint) { return; }

    m_armAngleSetpoint = angleSetpoint;
    setArmAngle();
  }


  public void setArmAngle() {
    // Angle setpoint in degrees
    double calculatedVoltage = m_armPIDController.calculate(
      getAngle(),
      m_armAngleSetpoint
    ) + ArmConstants.kArmStatic;
    // ) + m_armFeedforward.calculate(m_armAngleSetpoint, 0);

    // Arm voltaj sınırlaması
    if (calculatedVoltage > ArmConstants.kArmMaxVoltage) {
      calculatedVoltage = ArmConstants.kArmMaxVoltage;
    } else if (calculatedVoltage < -ArmConstants.kArmMaxVoltage) {
      calculatedVoltage = -ArmConstants.kArmMaxVoltage;
    }

    setVoltage(calculatedVoltage);
  }


  public void setVoltage(double voltage) {
    m_lastVoltage = voltage;
    setVoltage();
  }

  private void checkLimits() {
    // Eğer alt sınıra değiyorsak ve motor daha 
    // fazla aşağı indirmek istiyorsa izin verme
    if (m_armLowerLimit.get() == false && m_lastVoltage < ArmConstants.kArmStatic) {
      System.out.println("Arm motorlari alt sinira eristi, motorlarin calismasi engelleniyor");
      m_lastVoltage = ArmConstants.kArmStatic;
      setVoltage();
      return;
    }
    // Eğer üst sınıra değiyorsak ve motor daha 
    // fazla yukarı çıkarmak istiyorsa izin verme
    if (m_armUpperLimit.get() == false && m_lastVoltage > ArmConstants.kArmStatic) {
      System.out.println("Arm motorlari ust sinira eristi, motorlarin calismasi engelleniyor");
      m_lastVoltage = ArmConstants.kArmStatic;
      setVoltage();
      return;
    }
  }

  public void setVoltage() {
    checkLimits();
    m_armMotor1.setVoltage(-m_lastVoltage);
    m_armMotor2.setVoltage(-m_lastVoltage);
    m_armMotor3.setVoltage(-m_lastVoltage);
    m_armMotor4.setVoltage(-m_lastVoltage);
  }

  public double getAngle() {
    return -m_gyro.getAngle();
  }

  public double getRate() {
    return -m_gyro.getRate();
  }

  public Command armLowerReset() {
    return new FunctionalCommand(
      () -> { isPIDActive = false; },
      () -> { setVoltage(-4); },
      interrupted -> { setVoltage(0); },
      () -> m_armLowerLimit.get() == false
    );
  }


  public Command armUpperReset() {
    return new FunctionalCommand(
      () -> { isPIDActive = false; },
      () -> { setVoltage(5); },
      interrupted -> { setVoltage(0); },
      () -> m_armUpperLimit.get() == false
    );
  }


  public Command setArmAngleOnceCommand(DoubleSupplier angleSetpoint) {
    return runOnce(() -> setArmAngle(angleSetpoint.getAsDouble()));
  }
  
  

  public Command setArmAngleCommand(DoubleSupplier angleSetpoint) {
    return run(() -> setArmAngle(angleSetpoint.getAsDouble()));
  }


  // public Command increaseArmAngleCommand() {
  //   return runOnce(() -> {
  //     m_armAngleSetpoint++;
  //   });
  // }
  

  // public Command decreaseArmAngleCommand() {
  //   return runOnce(() -> {
  //     m_armAngleSetpoint--;
  //   });
  // }


  @Override
  public void periodic() {
    // Eğer kol alt limite değdiyse gyroyu sıfırla
    if (m_armLowerLimit.get() == false) {
      m_gyro.reset();
    }

    // pid aktifse arma değer ver
    if (isPIDActive) {
      setArmAngle();
    }

    checkLimits();
  }

  /**
   * Returns a command that will execute a quasistatic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
  //   return sysIdRoutine.quasistatic(direction);
  // }

  /**
   * Returns a command that will execute a dynamic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
  //   return sysIdRoutine.dynamic(direction);
  // }
}
