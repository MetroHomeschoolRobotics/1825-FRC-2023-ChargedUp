// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;


public class autoBalance extends CommandBase {

  private PIDController _PIDController = new PIDController(0.017, 0, 0.0025);
  private Drivetrain drivetrain;

  /** Creates a new autoBalance. */
  public autoBalance(Drivetrain _drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrain = _drivetrain;
    addRequirements(_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.autoDrive(-(_PIDController.calculate(drivetrain.getPitchAngle())), 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.driveMovement(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
