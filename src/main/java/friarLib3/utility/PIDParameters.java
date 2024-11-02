package friarLib3.utility;

import java.util.HashSet;
import java.util.Set;

import com.ctre.phoenix.motorcontrol.can.BaseTalon;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Represents a set of PID constants
 */
public class PIDParameters implements Sendable
{
    private int SlotIndex;

    private double P;
    private double I;
    private double D;
    private double F;
    private double IZone;

    private final Set<BaseTalon> linkedMotors  = new HashSet<>();
    
    public PIDParameters (int slotIdx, String sendableName, double p, double i, double d, double f, double iZone)
    {
        SlotIndex = slotIdx;
        P = p;
        I = i;
        D = d;
        F = f;
        IZone = iZone;
        
        SmartDashboard.putData(sendableName, this);
    }

    public PIDParameters (int slotIdx, String sendableName, double p, double i, double d)
    {
        this(slotIdx, sendableName, p, i, d, 0, 0);
    }

    /**
     * Helper method to initialize a Talon's PID parameters
     *
     * @param motor motor to configure
     */
    public void configureMotorPID (BaseTalon motor) {
        linkedMotors.add(motor);
        updateMotorPID();
    }

    public void updateMotorPID ()
    {
        for (BaseTalon linkedMotor : linkedMotors) {
            linkedMotor.config_kP(0, P);
            linkedMotor.config_kI(0, I);
            linkedMotor.config_kD(0, D);
            linkedMotor.config_kF(0, F);
            linkedMotor.config_IntegralZone(0, IZone);
        }
    }

	@Override
	public void initSendable(SendableBuilder builder)
    {
        builder.setSmartDashboardType("PIDController");
        builder.addDoubleProperty("p", this::GetP, this::SetP);
        builder.addDoubleProperty("i", this::GetI, this::SetI);
        builder.addDoubleProperty("d", this::GetD, this::SetD);
        builder.addDoubleProperty("f", this::GetF, this::SetF);
        builder.addDoubleProperty("iZone", this::GetIZone, this::SetIZone);
    }

    public int GetSlotIndex() { return SlotIndex; }
    public double GetP() { return P; }
    public double GetI() { return I; }
    public double GetD() { return D; }
    public double GetF() { return F; }
    public double GetIZone() { return IZone; }

    public void SetP(double p)
    {
        this.P = p;
        updateMotorPID();
    }

    public void SetI(double i)
    {
        this.I = i;
        updateMotorPID();
    }

    public void SetD(double d)
    {
        this.D = d;
        updateMotorPID();
    }

    public void SetF(double f)
    {
        this.F = f;
        updateMotorPID();
    }

    public void SetIZone(double iZone)
    {
        this.IZone = iZone;
        updateMotorPID();
    }
}