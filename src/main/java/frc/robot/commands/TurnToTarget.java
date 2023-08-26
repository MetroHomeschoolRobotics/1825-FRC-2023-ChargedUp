// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class TurnToTarget extends CommandBase {
  private Drivetrain drivetrain;
  private Limelight limelight;

  private PIDController pidTurn = new PIDController(0.03, 0, 0);

  private String pipeline;
  
  private double turnAngle;
  private double turnSpeed;

  /** Creates a new TurnToTarget. */
  public TurnToTarget(Drivetrain _drivetrain, Limelight _limelight, String _pipeline) {
    drivetrain = _drivetrain;
    limelight = _limelight;
    pipeline = _pipeline;
    addRequirements(_limelight);
    addRequirements(_drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (pipeline == "cone") {
      limelight.setToCone();
    } else if (pipeline == "cube") {
      limelight.setToCube();
    }
    
    pidTurn.setTolerance(1);
    pidTurn.enableContinuousInput(-180, 180);


    if(limelight.hasTargets()) {
      turnAngle = limelight.getTargetYaw();
    }else {
      turnAngle = 0;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(limelight.hasTargets()) {
      turnAngle = limelight.getTargetYaw();
    }

    turnSpeed = pidTurn.calculate(drivetrain.getHeading(), turnAngle);

    drivetrain.autoTurnDrive(turnSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.tankDriveVolts(0, 0);  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pidTurn.atSetpoint();
  }
}
