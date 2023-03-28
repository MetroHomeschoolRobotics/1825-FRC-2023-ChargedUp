// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TimeofFlight extends SubsystemBase {
  /** Creates a new TimeofFlight. */
  private TimeOfFlight grabberSensor = new TimeOfFlight(Constants.ToFSensor);
  
  public TimeofFlight() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ToFDistanceSensor", getSensorDistanceMM());
  }

  public long getSensorid() {
    return grabberSensor.getSerialNumber();
  }
  public double getSensorDistanceMM(){
    return grabberSensor.getRange();
  }
  public double getSensorDistanceCM(){
    return getSensorDistanceMM()/100;
  }
  public double getSensorDistanceMeters(){
    return getSensorDistanceMM()/1000;
  }
  public boolean isMeasurementSuccessful(){
    return grabberSensor.isRangeValid();
}
}