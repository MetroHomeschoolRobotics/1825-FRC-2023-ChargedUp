// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;


public class DriveTeleop extends CommandBase {

  private CommandXboxController xboxController;
  private XboxController xboxController2 = new XboxController(0);
  private Drivetrain _drivetrain;
  private Double forward;
  private Double turning;



  /** Creates a new DriveTeleop. */
  public DriveTeleop(Drivetrain drivetrain, CommandXboxController driverController) {
    // Use addRequirements() here to declare subsystem dependencies.
    _drivetrain = drivetrain;
    xboxController = driverController;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _drivetrain.resetHeading();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(xboxController2.getBButton() == true) {
      _drivetrain.autoBalance();
    }
    else{
      forward = -xboxController.getLeftY();
      turning = xboxController.getLeftX();
      _drivetrain.driveMovement(forward, turning);
      SmartDashboard.putNumber("forward", forward);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _drivetrain.driveMovement(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
