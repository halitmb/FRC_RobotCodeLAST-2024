// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants.IntakeConstans;
import frc.robot.Constants.PnumaticConstans;;

public class PnumaticSubsystem extends SubsystemBase {
  private final Compressor comp= new Compressor(0,PneumaticsModuleType.CTREPCM);
  private final Solenoid topSolenoid1 = new Solenoid(PneumaticsModuleType.CTREPCM, PnumaticConstans.selonoid1);
  private final Solenoid topSolenoid2 = new Solenoid(PneumaticsModuleType.CTREPCM, PnumaticConstans.selonoid2);

  /** Creates a new PnumaticSubsystem. */
  public PnumaticSubsystem() {

  }

  public void ClimbOn() {
    topSolenoid1.set(false);
    ;
    topSolenoid2.set(true);
  }

  public void climof() {
    topSolenoid1.set(true);
    ;
    topSolenoid2.set(false);
  }

  @Override

  public void periodic() {
    comp.enableDigital();
    // This method will be called once per scheduler run
  }
}
