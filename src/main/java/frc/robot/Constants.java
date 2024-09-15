// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double ROBOT_MASS = 66.4;
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final class AutonConstants {
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
    public static final PIDConstants ANGLE_PID   = new PIDConstants(0.4, 0, 0.01);
  }

  public static final class DrivebaseConstants {
    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants {
    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.06;
    public static final double LEFT_Y_DEADBAND  = 0.06;
    public static final double RIGHT_X_DEADBAND = 0.06;
    public static final double TURN_CONSTANT    = 6;
  }

  public static class IntakeConstans {
    public static final int kObjectSensorId = 1;

    public static final int kIntakeMotorId = 27;
    public static final int kFeederMotorId = 26;
    
    public static final double kIntakeVoltage = 10;
    public static final double kFeederVoltage = 10;
  }

  public static class ShooterConstants {
    public static final int kUpperShooterMotorId = 24;
    public static final int kLowerShooterMotorId = 25;
    // public static final double kShooterPower = 0.6;
    public static final double kShooterVoltage = 8;
    public static final double kShooterDelay = 1;

  }

  public static class PnumaticConstans   {
    public static final int comp = 0;
    public static final int selonoid1= 5;
    // public static final double kShooterPower = 0.6;
    public static final int selonoid2 = 6;
   

  }

  public static class ArmConstants {
    // public static final Measure<Velocity<Voltage>> kSysIdRampRate = Units.Volts.per(Units.Seconds).of(1);
    // public static final Measure<Voltage> kSysIdStepVoltage = Units.Volts.of(2);
    // public static final Measure<Time> kSysIdTimeout = Units.Seconds.of(4);

    public static final int kArmLowerSensorId = 2;
    public static final int kArmUpperSensorId = 0;

    public static final int kArmMotor1Id = 20;
    public static final int kArmMotor2Id = 21;
    public static final int kArmMotor3Id = 22;
    public static final int kArmMotor4Id = 23;


    public static final double kArmMaxVoltage = 5;
    public static final double kArmKp = 0.2;
    public static final double kArmKd = 0;
    public static final double kArmKi = 0.2;

    public static final double kArmStatic = 0.8;

    public static final double kArmKs = 0.1;
    public static final double kArmKg = 1.4;
    public static final double kArmKv = 0.03;
  }

  public static class ClimbConstants {
  }
  public static class targetConstans {
     public static final double TARGET_SPAKER_P = 0.02;
  public static final double TARGET_SPAKER_D = 0;
 
  }
}
