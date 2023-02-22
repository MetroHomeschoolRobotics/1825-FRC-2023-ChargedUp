// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Limelight extends SubsystemBase {

  PhotonCamera limelight = new PhotonCamera("limelight");

  private PhotonTrackedTarget target = limelight.getLatestResult().getBestTarget();

  private Drivetrain drivetrain;

  /** Creates a new Limelight. */
  public Limelight() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void getLatestResult(){
    limelight.getLatestResult();
  }
  public void getBestTarget(){
    limelight.getLatestResult().getBestTarget();
  }
  public double getTargetYaw(){
    return target.getYaw();
  }
  public double getTargetPitch(){
    return target.getPitch();
  }
  public double getTargetArea(){
    return target.getArea();
  }
  public double getTargetSkew(){
    return target.getSkew();
  }
  public int getAprilId(){
    return target.getFiducialId();
  }
  public double getTargetAmbiguity(){
    return target.getPoseAmbiguity();
  }
  public Transform3d getTargetPosition(){
    return target.getBestCameraToTarget();
  }
  // this distance is for the loading zone
  public double getLoadingAprilDistance(){
    return PhotonUtils.calculateDistanceToTargetMeters(RobotMap.limelightCameraHeightM, Units.inchesToMeters(23.375), Units.degreesToRadians(drivetrain.getHeading()), Units.degreesToRadians(getTargetPitch()));
  }
  // this distance is for the cube nodes
  public double getNodeAprilDistance(){
    return PhotonUtils.calculateDistanceToTargetMeters(RobotMap.limelightCameraHeightM, Units.inchesToMeters(14.25), Units.degreesToRadians(drivetrain.getHeading()), Units.degreesToRadians(getTargetPitch()));
  }

  public boolean hasTargests(){
    return limelight.getLatestResult().hasTargets();
  }
}
