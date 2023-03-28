// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class Drivetrain extends SubsystemBase {
  
  CANSparkMax motor1 = new CANSparkMax(1,MotorType.kBrushless);
  CANSparkMax motor2 = new CANSparkMax(2,MotorType.kBrushless);
  CANSparkMax motor3 = new CANSparkMax(3,MotorType.kBrushless);
  CANSparkMax motor4 = new CANSparkMax(4,MotorType.kBrushless);
  private final Field2d field = new Field2d();
  private AHRS gyro = new AHRS();
  private Pose2d position = new Pose2d();


  // this information could be of use in the future for distance tracking
  private static final double wheelRadiusInches = 3;
  private static final double gearRatio = 10.71;//10.71

  private DifferentialDrive difDrivetrain = new DifferentialDrive(motor1, motor3);
  private final DifferentialDriveOdometry odometry;



  /** Creates a new Drivetrain. */
  public Drivetrain() {
    // set the dead zone between the joysticks
    difDrivetrain.setDeadband(0.01);
    
    // inverts the right side 
    motor1.setInverted(true);
    motor3.setInverted(false);

    // causes the other motors to follow the original two motors
    motor2.follow(motor1);
    motor4.follow(motor3);
    
    // Set the conversion factor of the encoders
    motor1.getEncoder().setPositionConversionFactor((Units.inchesToMeters(wheelRadiusInches)*2*Math.PI)/(gearRatio));
    motor3.getEncoder().setPositionConversionFactor((Units.inchesToMeters(wheelRadiusInches)*2*Math.PI)/(gearRatio));

    motor1.getEncoder().setVelocityConversionFactor((Units.inchesToMeters(wheelRadiusInches)*2*Math.PI)/(gearRatio));
    motor3.getEncoder().setVelocityConversionFactor((Units.inchesToMeters(wheelRadiusInches)*2*Math.PI)/(gearRatio));
    
    motor1.setSmartCurrentLimit(35);
    motor2.setSmartCurrentLimit(35);
    motor3.setSmartCurrentLimit(35);
    motor4.setSmartCurrentLimit(35);
    
    
    // resets the gyro
    gyro.reset();
    gyro.calibrate();
    //reset all encoders
    resetEncoders();

    
    SmartDashboard.putData("field", field);
    
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(-getHeading()),motor3.getEncoder().getPosition(),motor1.getEncoder().getPosition()); 
    
    
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Encoder", motor1.getEncoder().getPosition());
    SmartDashboard.putNumber("Right Encoder", motor3.getEncoder().getPosition());
    SmartDashboard.putNumber("Encoder Difference", motor1.getEncoder().getPosition() - motor3.getEncoder().getPosition());
    SmartDashboard.putNumber("Distance", motor3.getEncoder().getPosition());
    SmartDashboard.putNumber("Velocity", motor3.getEncoder().getVelocity());
    SmartDashboard.putNumber("Heading", gyro.getAngle());
    SmartDashboard.putData(gyro);
    SmartDashboard.putNumber("Rotate Angle", gyro.getYaw());

    odometry.update(Rotation2d.fromDegrees(-getHeading()), getDistanceL(), getDistanceR());

    field.setRobotPose(odometry.getPoseMeters());
    // This method will be called once per scheduler run
  }

  public void resetEncoders() {
    motor1.getEncoder().setPosition(0);
    motor3.getEncoder().setPosition(0);
  }
  public void resetHeading() {
    gyro.reset();
  }
  public void resetOdometry(Pose2d position){
    resetEncoders();
    odometry.resetPosition(Rotation2d.fromDegrees(-getHeading()), motor3.getEncoder().getPosition(),motor1.getEncoder().getPosition(), position);
  }
  
  public Rotation2d getRotation2d() {//current heading in trajectory following format
    return Rotation2d.fromDegrees(-getHeading());
  }
  public Field2d getField2d(){
    return field;
  }
  public Pose2d getPosition() {
    return odometry.getPoseMeters();
  }
  public double getHeading() {
    return gyro.getAngle();
  }
  public double getPitchAngle() {
    // TODO I reversed this
    return gyro.getPitch()*-1;
  }
  public double getDistanceR() {
    return motor1.getEncoder().getPosition();
  }
  public double getDistanceL() {
    return motor3.getEncoder().getPosition();
  }
  public double getAverageEncoderDistance() {
    return getDistanceR()+getDistanceL()/2;
  }
  public DifferentialDriveWheelSpeeds getWheelSpeed(){
    return new DifferentialDriveWheelSpeeds(motor3.getEncoder().getVelocity(),motor1.getEncoder().getVelocity());
  }
  public double getTurnRate(){
    return gyro.getRate();
  }
  public void getSignal() {
    difDrivetrain.feed();
  }


  public void tankDriveVolts(double leftVolts, double rightVolts){
    motor1.set(rightVolts);
    motor3.set(leftVolts);
    difDrivetrain.feed();
  }
  public void autoDrive(double speed, double rotation) {
    motor1.set(speed-rotation);
    motor3.set(speed-rotation);
    difDrivetrain.feed();
  }
  public void autoTurnDrive(double speed){
    double speedR = speed;
    double speedL = -speed;
    motor1.set(speedR);
    motor3.set(speedL);
    difDrivetrain.feed();
  }
  public void driveMovement(double Xspeed, double Zrotation) {
    difDrivetrain.arcadeDrive(Xspeed, Zrotation, true);
  }
}
