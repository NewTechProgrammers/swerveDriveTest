package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstants;

public class TakingModule extends SubsystemBase
{
    private CANSparkMax smallMotor1;
    private CANSparkMax smallMotor2;
    private CANSparkMax bigMotor1;
    private RelativeEncoder turningEncoder;

    public CANSparkMax popychaniechyba;
    public CANSparkMax shoot2;
    public CANSparkMax shoot3;


    PIDController bigMotor1PidController = new PIDController(0.01, 0, 0);
    VictorSP actuator = new VictorSP(0);

    public TakingModule(int smallMotor1Id, int smallMotor2Id, int bigMotor1Id, int shoot1Id, int shoot2Id, int shoot3Id)
    {
        smallMotor1 = new CANSparkMax(smallMotor1Id, MotorType.kBrushless);
        smallMotor2 = new CANSparkMax(smallMotor2Id, MotorType.kBrushless);
        bigMotor1 = new CANSparkMax(bigMotor1Id, MotorType.kBrushless);
        popychaniechyba = new CANSparkMax(shoot1Id, MotorType.kBrushless);
        shoot2 = new CANSparkMax(shoot2Id, MotorType.kBrushless);
        shoot3 = new CANSparkMax(shoot3Id, MotorType.kBrushless);

    }

    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    public void onDPadUp()
    {
        bigMotor1.set(0.2);
    }

    public void onDPadDown()
    {
        bigMotor1.set(-0.2);
    }

    public void onLeftBumper()
    {
            smallMotor1.set(-0.5);
            smallMotor2.set(-0.5);
    }

    public void onRightBumper()
    {
            smallMotor1.set(0.5);
            smallMotor2.set(0.5);
    }

    public void onReleaseSmallMotor()
    {
        smallMotor1.set(0);
        smallMotor2.set(0);
    }

    public void onReleaseBigMotor()
    {
        bigMotor1.set(0);
    }

    public void onLeftTrigger()
    {
            popychaniechyba.set(-0.95);
    }

    public void onRightTrigger()
    {
            shoot2.set(-0.25);
            shoot3.set(-0.95);
    }

    double degreesToRadians(double degrees)
    {
        return (degrees * (Math.PI / 180));
    }

    public void onY()
    {
        actuator.set(1);
    }

    public void onX()
    {
        actuator.set(-1);
    }

    public void onReleaseActuator()
    {
        actuator.set(0);
    }

    public void onReleaseRightTrigger()
    {
        shoot2.set(0);
        shoot3.set(0);
    }

    public void onReleaseLeftTrigger()
    {
            popychaniechyba.set(0);
    }

}