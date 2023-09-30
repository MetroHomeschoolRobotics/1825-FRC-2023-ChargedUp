// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.TimeofFlight;

public class ExtendRotateArmCone extends CommandBase {

  private Arm arm;
  private Pneumatics grabber;
  private double extendDist = 0;
  private boolean finished = false;
  private TimeofFlight timeOfFlight;
  private double angle2;

  /** Creates a new ExtendRotateArmCone. */
  public ExtendRotateArmCone(Arm _arm, Pneumatics _grabber, TimeofFlight _TimeofFlight) {
    addRequirements(_arm);
    addRequirements(_grabber);

    arm = _arm;
    grabber = _grabber;
    timeOfFlight = _TimeofFlight;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //TODO returns NaN
    angle2 = Units.radiansToDegrees(Math.PI - (Math.acos(Units.metersToInches(Constants.GrabConeHeight/100)/arm.getTeleDistance())));
    
    SmartDashboard.putNumber("Angle Input", Units.metersToInches(Constants.GrabConeHeight/100)/arm.getTeleDistance());

    //arm.moveToTarget(Units.radiansToDegrees(angle2), extendDist);

    

    if(timeOfFlight.isMeasurementSuccessful() && timeOfFlight.getSensorDistanceMM() <= 260){
      grabber.setGrabberClose();
      finished = true;
    }else{
      extendDist += 0.5;
      //angle2-=0.07;
    }

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
