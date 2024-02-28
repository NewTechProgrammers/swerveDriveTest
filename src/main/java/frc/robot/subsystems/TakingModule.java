package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TakingModule extends SubsystemBase
{
    private CANSparkMax smallMotor1;
    private CANSparkMax smallMotor2;

    public TakingModule(int smallMotor1Id, int smallMotor2Id)
    {
        smallMotor1 = new CANSparkMax(smallMotor1Id, MotorType.kBrushless);
        smallMotor2 = new CANSparkMax(smallMotor2Id, MotorType.kBrushless);
    }


    public void onLeftBumper()
    {
            smallMotor1.set(-0.3);
            smallMotor2.set(-0.3);
    }

    public void onRightBumper()
    {
            smallMotor1.set(0.3);
            smallMotor2.set(0.3);
    }

    public void onRelease()
    {
        smallMotor1.set(0);
        smallMotor2.set(0);
    }

    public void onLeftTrigger()
    {
            smallMotor1.set(-0.5);
            smallMotor2.set(-0.5);
    }

    public void onRightTrigger()
    {
            smallMotor1.set(0.5);
            smallMotor2.set(0.5);
    }
}
