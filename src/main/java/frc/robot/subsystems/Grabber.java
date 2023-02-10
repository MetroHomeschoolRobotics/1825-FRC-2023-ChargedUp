// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Grabber extends SubsystemBase {

  private PneumaticHub Pneumatic1 = new PneumaticHub(1);
  private PneumaticHub pneumatic2 = new PneumaticHub(2);


  /** Creates a new Grabber. */
  public Grabber() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
