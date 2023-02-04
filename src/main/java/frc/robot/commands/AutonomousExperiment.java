// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class AutonomousExperiment extends CommandBase {

  private Drivetrain _drivetrain;           //outputs a number of distance          outputs how fast you're moving away
//                                            Probably must be below 1
  private PIDController _PIDController = new PIDController(0.0000000017, 0, .0025);

  Double distance;
  Double turnAngle;


  /** Creates a new AutonomousExperiment. */
  public AutonomousExperiment(Drivetrain drivetrain, double _distance, double _turnAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    distance = _distance;
    turnAngle = _turnAngle;
    _drivetrain = drivetrain;
    addRequirements(drivetrain);
  }




  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _drivetrain.resetHeading();
    _drivetrain.resetEncoders();
    _PIDController.setTolerance(2,5);
    _PIDController.setIntegratorRange(-.5, .5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Double turning = _PIDController.calculate(.5*(_drivetrain.getDistance()+_drivetrain.getDistance1()),distance);
    Double forward = MathUtil.clamp(_PIDController.calculate(.5*(_drivetrain.getDistance()+_drivetrain.getDistance1()), distance),-.5,.5);
    
    SmartDashboard.putNumber("Forward Speed", forward);
    System.out.println(turning);

    Double SDistance = _drivetrain.getDistance();
    
    //if(turnAngle != 0){
    //  while(_drivetrain.getHeading()<=turnAngle){
    //    _drivetrain.autoDrive(0, 0.1);
    //  }
    //}else if(distance != 0){
      //while(_drivetrain.getDistance()<=distance){
        _drivetrain.autoDrive(MathUtil.clamp(_PIDController.calculate(.5*(_drivetrain.getDistance()+_drivetrain.getDistance1()), distance),-.5,.5),
        _PIDController.calculate(_drivetrain.getHeading(), turnAngle));
        //}
    //}
    _drivetrain.getSignal();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _drivetrain.autoDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return _PIDController.atSetpoint();
  }
}
