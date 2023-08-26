// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class MoveToTarget extends CommandBase {

  private PIDController pidTurn = new PIDController(0.03, 0, 0);
  private PIDController pidFoward = new PIDController(0.3, 0, 0);

  private Drivetrain drivetrain;
  private Limelight limelight;
  private String pipeline;
  private double turnSpeed;

  private double forwardSpeed;
  private double forwardDistance;
  private double turnAngle;


  /** Creates a new MoveToTarget. */
  public MoveToTarget(Limelight _limelight, Drivetrain _drivetrain, String _pipeline) {
    addRequirements(_limelight);
    addRequirements(_drivetrain);

    limelight = _limelight;
    drivetrain = _drivetrain;
    pipeline = _pipeline;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // TODO changed this whole file. needs testing

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // set the pipeline to what you want

    if (pipeline == "cone") {
      limelight.setToCone();
    } else if (pipeline == "cube") {
      limelight.setToCube();
    } else if (pipeline == "april") {
      limelight.setToApril();
    } else if (pipeline == "reflective") {
      limelight.setToReflective();
    }
    pidFoward.setTolerance(0.001);
    pidTurn.setTolerance(1);
    pidTurn.enableContinuousInput(-180, 180);

    if(limelight.hasTargets()) {
      forwardDistance = limelight.getLoadingAprilDistance();
      turnAngle = limelight.getTargetYaw();
    }else {
      forwardDistance = 0;
      turnAngle = 0;
    }


    //drivetrain.resetHeading();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if(limelight.hasTargets()) {
      
      forwardDistance = limelight.getLoadingAprilDistance();
      turnAngle = limelight.getTargetYaw();

      System.out.println(forwardDistance);
    }



    if(pidTurn.atSetpoint() == false) {
      turnSpeed = pidTurn.calculate(drivetrain.getHeading(), turnAngle);
    }else{
      //turnSpeed = 0;
      forwardSpeed = pidFoward.calculate(drivetrain.getAverageEncoderDistance(),forwardDistance-1);
    }
    

    drivetrain.driveMovement(forwardSpeed, turnSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println(pidFoward.atSetpoint());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (pidFoward.atSetpoint()) {
      return true;
    } else {
      return false;
    }
  }
}
