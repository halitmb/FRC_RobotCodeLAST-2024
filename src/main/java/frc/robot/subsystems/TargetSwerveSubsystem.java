// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.targetConstans;;

public class TargetSwerveSubsystem extends SubsystemBase {
  PhotonCamera camera;
  PhotonCamera obje;
 PIDController turnController = new PIDController(targetConstans.TARGET_SPAKER_P, 0, targetConstans.TARGET_SPAKER_D  );

  /** Creates a new TargetSwerveSubsystem. */
  public TargetSwerveSubsystem() { 
    camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
  obje = new PhotonCamera("TANDBERG_Video");}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
   public boolean TargetAccept(boolean deneme) {
  
    return deneme;
  }
    public double TargetToSpeaker() {
    var result = camera.getLatestResult();

    if (result.hasTargets()&&TargetAccept(true)) {
      PhotonTrackedTarget target = result.getBestTarget();
    

     
        // Cm cinsinden almak için 100 ile çarpıyorum
        double distance = target.getBestCameraToTarget().getX() * 100;
        SmartDashboard.putNumber("Speaker Distance", distance);
         double MoveY = turnController.calculate(result.getBestTarget().getYaw(), 0);
        return MoveY;
      
    }
    return -1;
  }
}
