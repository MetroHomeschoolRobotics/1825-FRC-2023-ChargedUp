// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ExtendToPoint extends CommandBase {

  private double kp=0.01;
  private double ki=0;
  private double kd=0;



  private PIDController pidExtend = new PIDController(kp, ki, kd);
  private Arm arm;
  private double exSetpoint;
  /** Creates a new ExtendToPoint. */
  public ExtendToPoint(Arm _arm, double setpoint){
    exSetpoint = setpoint;
    arm = _arm;
    addRequirements(_arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pidExtend.atSetpoint();
  }
}
