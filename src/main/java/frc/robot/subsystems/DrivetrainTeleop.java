// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;



class MotorWrapper implements MotorController {
  private final CANSparkMax _motor;

  public MotorWrapper(CANSparkMax motor) {
    _motor = motor;
  }
  public void disable() {
    _motor.set(0);
  }
  public double get() {
    return _motor.get();
  }
  public boolean getInverted() {
    return _motor.getInverted();
  }
  public void set(double speed) {
    _motor.set(speed);
  }
  public void setInverted(boolean inverted) {
    _motor.setInverted(inverted);
  }
  public void setVoltage(double voltage) {
    _motor.setVoltage(voltage);
  }
  public void stopMotor() {
    set(0.0);
  }
}




public class DrivetrainTeleop extends SubsystemBase {

  // create the variables for the OOP of drive-controller, xbox controller, and motor controllers
  private DifferentialDrive m_myRobot;
  private CommandXboxController m_controller = null;
 
  
  private MotorWrapper leftFrontMotor = null;  //the separate motor controllers
  private MotorWrapper leftRearMotor = null;
  private MotorWrapper rightFrontMotor = null;
  private MotorWrapper rightRearMotor = null;
  private MotorController m_leftMotor = null;
  private MotorController m_rightMotor = null;

  /** Creates a new DrivetrainTeleop. */
  public DrivetrainTeleop() {
    // set the variables above to their purpose
    leftFrontMotor = new MotorWrapper(new CANSparkMax(1,MotorType.kBrushless));
    leftRearMotor = new MotorWrapper(new CANSparkMax(2,MotorType.kBrushless));
    rightFrontMotor = new MotorWrapper(new CANSparkMax(3,MotorType.kBrushless));
    rightRearMotor = new MotorWrapper(new CANSparkMax(4,MotorType.kBrushless));
    m_leftMotor = new MotorControllerGroup(leftFrontMotor, leftRearMotor);
    m_rightMotor = new MotorControllerGroup(rightFrontMotor, rightRearMotor);
  }


  
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.setInverted(true);

    //create the drives (tank, arcade, curvature) and the controller with which you will control the robot
    m_myRobot = new DifferentialDrive(m_rightMotor, m_leftMotor);
    m_controller = new CommandXboxController(0);
    
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    // here we set the drive of the robot, using the X and Y axis of the left joystick (or in the case of the 
    // tank drive, the Y's of both joysticks)

    //m_myRobot.tankDrive(m_controller.getLeftY(), m_controller.getRightY());
    //m_myRobot.arcadeDrive(m_controller.getLeftY(),m_controller.getLeftX());

    // This is the best drive for the robot currently ☺
    final double speed = 1;
    final double controllerYSpeed = m_controller.getLeftY()*speed;
    final double controllerXSpeed = m_controller.getLeftX()*speed;

    m_myRobot.arcadeDrive(controllerYSpeed, controllerXSpeed);
  }
}
