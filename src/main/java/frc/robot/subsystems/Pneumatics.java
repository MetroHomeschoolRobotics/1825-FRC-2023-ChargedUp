// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase {

  private DoubleSolenoid grabberPneumatic1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
  private DoubleSolenoid grabberPneumatic2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 3, 4);


  /** Creates a new Grabber. */
  public Pneumatics() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.getBoolean("Grabber Open", getGrabberState());
  }

  public void changeGrabberState() {
    grabberPneumatic1.toggle();
    grabberPneumatic2.toggle();
  }
  public Boolean getGrabberState() {
    return grabberPneumatic1.isRevSolenoidDisabled();
  }


}
