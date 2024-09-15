// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class VisionSubsystem extends SubsystemBase {
  // PhotonCamera camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
  // PhotonCamera obje = new PhotonCamera("TANDBERG_Video");
   PhotonCamera camera;
  PhotonCamera obje;


  int targetID = 0;

  public VisionSubsystem() {
    camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
    obje = new PhotonCamera("TANDBERG_Video");
  }

  public boolean isSpeakerAvailable() {
    return getDistanceToSpeaker() > 0;
  }

  public double getDistanceToSpeaker() {
    var result = camera.getLatestResult();

    if (result.hasTargets()) {
      PhotonTrackedTarget target = result.getBestTarget();
      targetID = target.getFiducialId();

      if (targetID == 4 || targetID == 7) {
        // Cm cinsinden almak için 100 ile çarpıyorum
        double distance = target.getBestCameraToTarget().getX() * 100;
        SmartDashboard.putNumber("Speaker Distance", distance);
        
        return distance;
      }
    }
    return -1;
  }

  public double getPitchToSpeaker() {
    var result = camera.getLatestResult();

    if (result.hasTargets()) {
      PhotonTrackedTarget target = result.getBestTarget();
      targetID = target.getFiducialId();

      if (targetID == 4 || targetID == 7) {
        // Cm cinsinden almak için 100 ile çarpıyorum
        double pitch = target.getPitch();
        SmartDashboard.putNumber("Speaker Pitch", pitch);
        
        return pitch;
      }
    }
    return 0;
  }

  public boolean isAmpAvailable() {
    return getDistanceToAmp() > 0;
  }

  public double getDistanceToAmp() {
    var result = camera.getLatestResult();

    if (result.hasTargets()) {
      PhotonTrackedTarget target = result.getBestTarget();
      targetID = target.getFiducialId();

      if (targetID == 5) {
        // Cm cinsinden almak için 100 ile çarpıyorum
        double distance = target.getBestCameraToTarget().getX() * 100;
        SmartDashboard.putNumber("Amp Distance", distance);
        return distance;
      }
    }
    else{

    }
    return -1;
  }
}
