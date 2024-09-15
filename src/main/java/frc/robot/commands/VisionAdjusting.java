package frc.robot.commands;

import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;

public class VisionAdjusting {
    VisionSubsystem m_vision;
    SwerveSubsystem m_swerve;

    public VisionAdjusting(SwerveSubsystem swerve, VisionSubsystem vision) {
        m_swerve = swerve;
        m_vision = vision;
    } 
  
}
