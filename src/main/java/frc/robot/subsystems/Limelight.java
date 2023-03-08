// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Limelight extends SubsystemBase {

  PhotonCamera limelight = new PhotonCamera("OV5647");

  
  private NetworkTable networkTable = NetworkTableInstance.getDefault().getTable("Limelight");
  //private NetworkTable coloredShape = NetworkTableInstance.getDefault().getTable("ColoredShape");
  
  private PhotonTrackedTarget target = limelight.getLatestResult().getBestTarget();

  private Drivetrain drivetrain;

  /** Creates a new Limelight. */
  public Limelight(Drivetrain _drivetrain) {
    drivetrain = _drivetrain;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }


  // This changes the pipeline
  public void setToApril(){
    limelight.setPipelineIndex(0);
  }
  public void setToReflective(){
    limelight.setPipelineIndex(1);
  }
  public void setToShape(){
    limelight.setPipelineIndex(2);
  }
  public int getPipeline(){
    return limelight.getPipelineIndex();
  }

  public VisionLEDMode getLightState(){
    return limelight.getLEDMode();
  }
  public void getLatestResult(){
    limelight.getLatestResult();
  }
  public void getBestTarget(){
    limelight.getLatestResult().getBestTarget();
  }
  public double getTargetYaw(){
    return limelight.getLatestResult().getBestTarget().getYaw();
  }
  public double getTargetPitch(){
    return limelight.getLatestResult().getBestTarget().getPitch();
  }
  public double getTargetArea(){
    return limelight.getLatestResult().getBestTarget().getArea();
  }
  public double getTargetSkew(){
    return limelight.getLatestResult().getBestTarget().getSkew();
  }
  public int getAprilId(){
    return limelight.getLatestResult().getBestTarget().getFiducialId();
  }
  public double getTargetAmbiguity(){
    return limelight.getLatestResult().getBestTarget().getPoseAmbiguity();
  }
  public Translation2d getLoadingTranslation(){
    return PhotonUtils.estimateCameraToTargetTranslation(getLoadingAprilDistance(), Rotation2d.fromDegrees(-getTargetYaw()));
  }
  public Translation2d getNodeTranslation(){
    return PhotonUtils.estimateCameraToTargetTranslation(getNodeAprilDistance(), Rotation2d.fromDegrees(-getTargetYaw()));
  }

  
  // this distance is for the loading zone
  public double getLoadingAprilDistance(){
    return PhotonUtils.calculateDistanceToTargetMeters(RobotMap.limelightCameraHeightM, Units.inchesToMeters(23.375), Units.degreesToRadians(drivetrain.getHeading()), Units.degreesToRadians(getTargetPitch()));
  }
  // this distance is for the cube nodes
  public double getNodeAprilDistance(){
    return PhotonUtils.calculateDistanceToTargetMeters(RobotMap.limelightCameraHeightM, Units.inchesToMeters(14.25), Units.degreesToRadians(drivetrain.getHeading()), Units.degreesToRadians(getTargetPitch()));
  }
  public void LEDOn(){
    limelight.setLED(VisionLEDMode.kOn);
  }
  public void LEDOff(){
    limelight.setLED(VisionLEDMode.kOff);
  }

  public boolean hasTargests(){
    return limelight.getLatestResult().hasTargets();
  }
}
