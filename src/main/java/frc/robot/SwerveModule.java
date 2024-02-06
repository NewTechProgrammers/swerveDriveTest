package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.*;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
    private CANSparkMax driveMotor;
    private CANSparkMax turningMotor;

    private RelativeEncoder driveEncoder;
    private RelativeEncoder turningEncoder;

    private PIDController turningPidController;

    private CANcoder absoluteEncoder;
    private boolean absoluteEncoderReversed;
    private double absoluteEncoderOffsetRad;
    
    //private final CANSparkMax driveMotor;
    //private final CANSparkMax turningMotor;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
                    int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed)
    {
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;

        absoluteEncoder = new CANcoder(absoluteEncoderId);

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = driveMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);

        turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);

        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    public double getDrivePosition(){
        return driveEncoder.getPosition();
    }

    public double getTurningPosition(){
        return turningEncoder.getPosition();
    }

    public double getDriveVelocity(){
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity(){
        return turningEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad(){
        double angle = Double.valueOf(absoluteEncoder.getSupplyVoltage().toString().substring(0,4)) / RobotController.getVoltage5V();
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders(){
        driveEncoder.setPosition(0); 
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }
    
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state){
        if(Math.abs(state.speedMetersPerSecond)<0.001){
            stop();
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        //SmartDashboard.putString("Swerve[" + absoluteEncoder.get + "] state", state.toString()); TODO : find .getChannel() in cancoder 
        // https://www.chiefdelphi.com/t/how-to-get-radians-voltage-from-ctre-cancoder/450759
        // https://www.chiefdelphi.com/t/0-to-auto-swerve-analoginput-help/406510
    }
    
    public void stop(){
        driveMotor.set(0);
        turningMotor.set(0);
    }

}
