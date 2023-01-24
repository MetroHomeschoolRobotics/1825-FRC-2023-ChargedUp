// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  
  CANSparkMax motor1 = new CANSparkMax(1,MotorType.kBrushless);
  CANSparkMax motor2 = new CANSparkMax(2,MotorType.kBrushless);
  CANSparkMax motor3 = new CANSparkMax(3,MotorType.kBrushless);
  CANSparkMax motor4 = new CANSparkMax(4,MotorType.kBrushless);

  // this information could be of use in the future for distance tracking
  private static final double wheelRadiusInches = 3;
  private static final double gearRatio = 8.46;


  private DifferentialDrive difDrivetrain = new DifferentialDrive(motor1, motor3);




  /** Creates a new Drivetrain. */
  public Drivetrain() {
    // set the dead zone between the joysticks
    difDrivetrain.setDeadband(0.01);
    
    // inverts the right side 
    motor1.getInverted();

    // causes the other motors to follow the original two motors
    motor2.follow(motor1);
    motor4.follow(motor3);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void driveMovement(double joystickX, double joystickY) {
    difDrivetrain.arcadeDrive(joystickX, joystickY, true);
  }



}
