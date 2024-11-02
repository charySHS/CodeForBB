package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.vision.Vision;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class PoseManager
{
    enum EPose
    {
        None,
        Stowed,
        Intake,
        Amp,
        Speaker,
        PreClimb,
        Source
    }

    private ArmSubsystem Arm;
    private IntakeSubsystem Intake;



    public PoseManager(ArmSubsystem arm, IntakeSubsystem intake)
    {
        Arm = arm;
        Intake = intake;
    }

    public EPose GetPoseForCurrentTag()
    {
        var target = Vision.GetBestTarget();
        if (target == null) { return EPose.None; }

        if (target.fiducialID == 4 || target.fiducialID == 7)
        {
            System.out.println("SpeakerPose");
            return EPose.Speaker;
        }
        else if (target.fiducialID == 5 || target.fiducialID == 6)
        {
            System.out.println("AmpPose");
            return EPose.Amp;
        }
        else if (target.fiducialID >= 11 || target.fiducialID <= 16)
        {
            System.out.println("Climb");
            return EPose.PreClimb;
        }
        System.out.println("No Pose");
        return EPose.None;
    }

    public Command Command_GoToPose(EPose pose)
    {
        if (pose == EPose.None) { return Commands.none(); }

        if (pose == EPose.Stowed)
        {
            return Commands.parallel(
                Intake.Command_SetPivotPosition(IntakeSubsystem.EPivotPosition.Stowed),
                Commands.waitSeconds(0.1)
                    .andThen(Arm.Command_SetPosition(ArmSubsystem.EArmPosition.Stowed))

            );
        }

        if (pose == EPose.Intake)
        {
            return Commands.parallel(
                Arm.Command_SetPosition(ArmSubsystem.EArmPosition.Stowed),
                Intake.Command_SetPivotPosition(IntakeSubsystem.EPivotPosition.Intake)
            );
        }

        if (pose == EPose.Amp)
        {
            return Commands.parallel(
                Intake.Command_SetPivotPosition(IntakeSubsystem.EPivotPosition.Amp),
                Commands.waitSeconds(0.1)
                    .andThen(Arm.Command_SetPosition(ArmSubsystem.EArmPosition.Amp))
            );


        }

        if (pose == EPose.Speaker)
        {
            return Commands.parallel(
                    Arm.Command_SetPosition(ArmSubsystem.EArmPosition.Shoot_speaker),
                    Intake.Command_SetPivotPosition(IntakeSubsystem.EPivotPosition.Shoot_speaker)
            );

        }

        if (pose == EPose.PreClimb)
        {
            return Commands.sequence(
                Arm.Command_SetPosition(ArmSubsystem.EArmPosition.Climb_FirstPos),
                Intake.Command_SetPivotPosition(IntakeSubsystem.EPivotPosition.Climb)
            );
        }

        if (pose == EPose.Source)
        {
            return Commands.sequence(
                    Arm.Command_SetPosition(ArmSubsystem.EArmPosition.Source),
                    Intake.Command_SetPivotPosition(IntakeSubsystem.EPivotPosition.Source)
            );
        }

        return Commands.none();

    }


    public Command Command_AutoPose()
    {
        return Command_GoToPose(GetPoseForCurrentTag());
    }

}
