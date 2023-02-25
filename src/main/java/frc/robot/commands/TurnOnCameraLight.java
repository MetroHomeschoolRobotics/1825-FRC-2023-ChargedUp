// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;

public class TurnOnCameraLight extends CommandBase {
  private Limelight limelight;
  /** Creates a new TurnOnCameraLight. */
  public TurnOnCameraLight(Limelight _limelight) {
    limelight = _limelight;
    addRequirements(_limelight);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(limelight.getLightState());
    // if(limelight.getLightState() == VisionLEDMode.kOff){
    //   limelight.LEDOn();
    // }else if(limelight.getLightState() == VisionLEDMode.kOn){
    //   limelight.LEDOff();
    // }else if(limelight.getLightState() == VisionLEDMode.kDefault){
    //   limelight.LEDOn();
    // }
    limelight.LEDOn();
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
