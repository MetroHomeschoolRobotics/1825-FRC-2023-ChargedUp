// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import org.photonvision.PhotonCamera;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;


public class driveToAprilTag extends CommandBase {
    // Constants such as camera and target height stored. Change per robot and goal!

    final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(14);

    final double TARGET_HEIGHT_METERS = Units.inchesToMeters(17);

    // Angle between horizontal and the camera.

    final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);


    // How far from the target we want to be

    final double GOAL_RANGE_METERS = Units.feetToMeters(8);


    // Change this to match the name of your camera

    PhotonCamera camera = new PhotonCamera("limelight");
        // PID constants should be tuned per robot
        
        final double LINEAR_P = 0.0000000017;

        final double LINEAR_D = 0.0025;
    
        PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);
    
    
        final double ANGULAR_P = 0.1;//working on finding
    
        final double ANGULAR_D = 0.025;//working on finding


    XboxController xboxController = new XboxController(0);
    Vision v = new Vision();
    PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);
    private Drivetrain drivetrain;
   public driveToAprilTag(Drivetrain _drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrain = _drivetrain;
    addRequirements(_drivetrain);
   }
    @Override

    public void execute() {

        double forwardSpeed;

        double rotationSpeed;


        forwardSpeed = -xboxController.getRightY();


            // Vision-alignment mode

            // Query the latest result from PhotonVision

            var result = camera.getLatestResult();


            if (result.hasTargets()) {

                // Calculate angular turn power

                // -1.0 required to ensure positive PID controller effort _increases_ yaw

                rotationSpeed = -turnController.calculate(result.getBestTarget().getYaw(), 0);

            } else {

                // If we have no targets, stay still.

                rotationSpeed = 0;

            }



        // Use our forward/turn speeds to control the drivetrain

        drivetrain.arcadeDrive(forwardSpeed, rotationSpeed);

    }

}