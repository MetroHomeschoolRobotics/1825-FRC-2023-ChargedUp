// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.Num;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.TimeofFlight;

public class ExtendRotateArmCone extends CommandBase {

  private Arm arm;
  private Pneumatics grabber;
  private double extendDist = 0;
  private boolean finished = false;
  private TimeofFlight timeOfFlight;
  private Drivetrain drivetrain;
  private double angle2;
  private double NumInACos;

  /** Creates a new ExtendRotateArmCone. */
  public ExtendRotateArmCone(Arm _arm, Pneumatics _grabber, TimeofFlight _TimeofFlight, Drivetrain _drivetrain) {
    addRequirements(_arm);
    addRequirements(_grabber);
    addRequirements(_drivetrain);

    arm = _arm;
    grabber = _grabber;
    timeOfFlight = _TimeofFlight;
    drivetrain = _drivetrain;

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
    // Angle2 = pi - ArcCos(HeightOfWhereToGrabCone / TelescopingDist)
    NumInACos = Units.metersToInches(Constants.GrabConeHeight/100)/arm.getTeleDistance();
    angle2 = Units.radiansToDegrees(Math.PI - (Math.acos(NumInACos)));
    
    SmartDashboard.putNumber("Angle Input", angle2);
    SmartDashboard.putNumber("GrabbingConeHeight/TelescopingDist", NumInACos);
    //System.out.println("Soludos Como estas!!!!!");

    arm.moveToTarget(130, 20);

    

    if(timeOfFlight.isMeasurementSuccessful() && timeOfFlight.getSensorDistanceMM() <= 450){//300
      grabber.setGrabberClose();
      finished = true;
    }else{
      //extendDist += 0.5;
      //angle2-=0.07;

      drivetrain.autoDrive(0.2, 0);
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
