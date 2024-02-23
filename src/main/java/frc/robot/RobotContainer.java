package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoysticksCmd;

import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
    
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

    private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);

    public RobotContainer(){
        swerveSubsystem.setDefaultCommand(new SwerveJoysticksCmd(
            swerveSubsystem,
            () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
            () -> driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
            () -> driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
            () -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));

        configureButtonBindings();
        Shuffleboard.getTab("Example tab").add(swerveSubsystem.gyro);
    }

    private void configureButtonBindings(){
        //new JoystickButton(driverJoystick, 2).whenPressed(() -> swerveSubsystem.zeroHeading());
        //new JoystickButton(driverJoystick, 2).onTrue(() -> );
        new JoystickButton(driverJoystick, 1).onTrue(swerveSubsystem.runOnce(swerveSubsystem::wheelZeroing));
    }

    // public Command wheelZeroingCommand(){
    //     swerveSubsystem.wheelZeroing();
    //     return 
    // }

    public Command getAutonomousCommand(){
        //1. Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        DriveConstants.kPhysicalMaxSpeedMetersPerSecond,
        DriveConstants.kPhysicalMaxAccelerationMetersPerSecondSquared)
            .setKinematics(DriveConstants.kDriveKinematics)
            .addConstraint(null);
        //2. Generate trajectory
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
            List.of(
                    new Translation2d(1, 0),
                    new Translation2d(1, -1)
            ),
            new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
            trajectoryConfig
        );
        //3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(DriveConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(DriveConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
            DriveConstants.kPThetaController,0,0,DriveConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        //4. Construct command to follow trajectory
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            trajectory,
            swerveSubsystem::getPose,
            DriveConstants.kDriveKinematics,
            xController,
            yController,
            thetaController,
            swerveSubsystem::setModuleStates, 
            swerveSubsystem);
        //5. add some init and wrap up and return everything    
        return new SequentialCommandGroup(
            new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
            swerveControllerCommand,
            new InstantCommand(() -> swerveSubsystem.stopModules())
        );
    }
}
