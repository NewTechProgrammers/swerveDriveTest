package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.geometry.Rotation2d;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveModuleState;


import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;

import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class SwerveModule {

    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final PIDController turningPidController;
    //private final SparkPIDController turningPidController;

    private final CANcoder absoluteEncoder;

    //private final AnalogInput absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        //absoluteEncoder = new AnalogInput(absoluteEncoderId);
        absoluteEncoder = new CANcoder(absoluteEncoderId, "rio");

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        
        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);

        turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }

        driveMotor.burnFlash();
        turningMotor.burnFlash();

        System.out.println("Turn conv factor pos: " + ModuleConstants.kTurningEncoderRot2Rad);
        System.out.println("Turn conv factor vel: " + ModuleConstants.kTurningEncoderRPM2RadPerSec);

        turningPidController = new PIDController(ModuleConstants.kPTurning, ModuleConstants.kITurning, ModuleConstants.kDTurning);
        
        //turningPidController = turningMotor.getPIDController();
        //turningPidController.setP(ModuleConstants.kPTurning); turningPidController.setI(ModuleConstants.kITurning); turningPidController.setD(ModuleConstants.kDTurning);
        //turningPidController.input
        
        //turningPidController.setPositionPIDWrappingMaxInput()
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);


        resetEncoders();
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad() {
        //double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();

        double angle = absoluteEncoder.getPosition().getValueAsDouble();
        
        angle *= 2.0 * Math.PI;
        
    
        angle -= absoluteEncoderOffsetRad;

        

        System.out.println("ANGLE: " + String.valueOf(angle) + " ABSOLUTE POS: " + absoluteEncoder.getPosition().getValueAsDouble());
        System.out.println("ANGLE return: " + angle * (absoluteEncoderReversed ? -1.0 : 1.0));
        
        // // SmartDashboard.putString("Angle Rad: ", String.valueOf(angle));

        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public double getAbsoluteEncoderPos(){
        return absoluteEncoder.getPosition().getValueAsDouble();
    }


    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());

        System.out.println("TURNING ENCODER: " + String.valueOf(turningEncoder.getPosition()));
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        //turningMotor.set(turningPidController.setReference())

        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getDeviceID() + "] state", state.toString());
        //SmartDashboard.putString("CANcoder[" + absoluteEncoder.getDeviceID() + "] + angle ", String.valueOf(absoluteEncoder.getPosition()) + " " +String.valueOf((absoluteEncoder.getPosition().getValueAsDouble()*360)%360) );
        
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }

    public void wheelZero(){
        //turningMotor.set(turningPidController.calculate(getTurningPosition(), 0));

        turningMotor.set(turningPidController.calculate(absoluteEncoder.getPosition().getValueAsDouble() * 21.428,0));
    }
}