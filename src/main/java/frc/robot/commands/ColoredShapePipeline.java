// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;

public class ColoredShapePipeline extends CommandBase {
  private Limelight limelight;
  private String _shape;
  /** Creates a new ColoredShapePipeline. */
  public ColoredShapePipeline(Limelight _limelight, String shape) {
    limelight = _limelight;
    _shape = shape;

    addRequirements(_limelight);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(_shape == "cone"){
      limelight.setToCone();
    }else if(_shape == "cube"){
      limelight.setToCube();
    }
    
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
