// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class MoveToTarget extends CommandBase {

  private PIDController pidTurn = new PIDController(0.09, 0, 0);
  private PIDController pidFoward = new PIDController(0.2, 0, 0);



  private Drivetrain drivetrain;
  private Limelight limelight;
  private String pipeline;
  private double turnSpeed;
  private boolean finishedTurning = false;

  private double forwardSpeed;

  /** Creates a new MoveToTarget. */
  public MoveToTarget(Limelight _limelight,Drivetrain _drivetrain, String _pipeline) {
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

    if(pipeline == "cone"){
      limelight.setToCone();
    }else if(pipeline == "cube"){
      limelight.setToCube();
    } else if(pipeline == "april"){
      limelight.setToApril();
    }else if(pipeline == "reflective"){
      limelight.setToReflective();
    }
    pidFoward.setTolerance(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(limelight.hasTargests()){
      forwardSpeed = -pidFoward.calculate(limelight.getLoadingAprilDistance(), 1);
      turnSpeed = -pidTurn.calculate(limelight.getTargetYaw(), 0);
      System.out.println(limelight.getLoadingAprilDistance());
    }else{
      forwardSpeed = 0;
      turnSpeed = 0;
    }


    drivetrain.driveMovement(forwardSpeed, turnSpeed);
    if(pidTurn.atSetpoint() == true){
        System.out.println("true");
    }
    //drivetrain.autoTurnDrive(turnSpeed);
    //drivetrain.tankDriveVolts(forwardSpeed, forwardSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(limelight.hasTargests() == true){
      return pidFoward.atSetpoint();
      
      }
      else{
        return true;
      }
  }
}
