// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class AutonomousExperiment extends CommandBase {

  private Drivetrain _drivetrain;

  Double distance;
  Double turnAngle;
  Double speed;


  /** Creates a new AutonomousExperiment. */
  public AutonomousExperiment(Drivetrain drivetrain,double _speed, double _distance, double turnAngleAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    distance = _distance;
    turnAngle = turnAngleAngle;
    speed = _speed;
    _drivetrain = drivetrain;
    addRequirements(drivetrain);
  }




  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _drivetrain.resetHeading();
    _drivetrain.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(distance == null){
      while(_drivetrain.getHeading() <= turnAngle){
        _drivetrain.driveMovement(0, speed);
      }
    }else if(turnAngle == null){
      while(_drivetrain.getDistance() <= distance){
        _drivetrain.driveMovement(speed, 0);
      }
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
