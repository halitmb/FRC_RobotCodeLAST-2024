// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;


/** An example command that uses an example subsystem. */
public class ShootCommand extends Command{ 
  private final IntakeSubsystem m_intake;
  private final ShooterSubsystem m_shooter;
  private boolean isShooterInitialized = false;
  private final Timer m_shooterTimer = new Timer();

  public ShootCommand(IntakeSubsystem intake, ShooterSubsystem shooter)  {
    m_intake = intake;
    m_shooter = shooter;

    addRequirements(intake);
    addRequirements(shooter);

    // aaaaddCommands(m_intake.runIntakeInOnceCommand());
    // addCommands(new WaitCommand(2));
    // addCommands(m_shooter.runShooterCommand());
  }

  @Override
  public void initialize() {
    isShooterInitialized = false;
    m_shooterTimer.stop();
    m_shooterTimer.reset();
  }

  @Override
  public void execute() {
    // Eğer shooter çalıştırılmmadıysa ve obje görüyorsak objeyi geri çek
    if (m_intake.getSensorReading() == false && isShooterInitialized == false) {
      System.out.println("aaa");
      // Shooterı da kontrol etme sebebim, eninde 
      // sonunda obje bu sensör üzerinden geçecek, 
      // her seferinde tekrar geri almaya çalışmasın diye
      m_intake.runIntakeOut();
      // m_intake.stopIntake();
      return;
    }

    // obje görmüyorsak ve shooter başlamadıysa başlat
    if (m_intake.getSensorReading() == true && isShooterInitialized == false) {
      m_shooterTimer.start();
      System.out.println("Shooter calistiriliyor...");
      m_shooter.runShooter();
      m_intake.runFeederIn();
      m_intake.stopIntake();
      isShooterInitialized = true;
      return;
    }

    // shooter başlayalı 1 sn olduysa intake de çalıştır
    if (m_shooterTimer.hasElapsed(ShooterConstants.kShooterDelay) && isShooterInitialized) {
      m_intake.runIntakeIn();
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.stopShooter();
    m_intake.stopIntake();
    m_intake.stopFeeder();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
