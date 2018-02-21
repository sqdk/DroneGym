using UnityEngine;
using System.Collections;

public class DronePidController : MonoBehaviour
{
    private ControlInterface controlInterface;
    private PidController[] VelocityControllers; // Order: X Y Z
    private PidController[] PositionControllers; 
    private int ticksSinceLastUpdate;
    private Vector3 VelocityTargets;

    public float VerticalVelocityTarget = 0;
    public float PitchAngleTarget = 0;
    public float RollAngleTarget = 0;
    public Vector3 PositionHoldTarget = new Vector3(0,10,0);
    public bool PositionHoldEnabled = true;
    public int TicksPerUpdate;

    [Header("Vertical PID Settings")]
    public double YProportionalGain = 2;
    public double YIntegralGain = 3;
    public double YDerivativeGain = 40;
    public double YOutputMax = 100;
    public double YOutputMin = 0;

    [Header("Pitch axis PID Settings")]
    public double PitchProportionalGain = 2;
    public double PitchIntegralGain = 0;
    public double PitchDerivativeGain = 32;
    public double PitchOutputMax = 25;
    public double PitchOutputMin = -25;

    [Header("Roll axis PID Settings")]
    public double RollProportionalGain = 2;
    public double RollIntegralGain = 0;
    public double RollDerivativeGain = 32;
    public double RollOutputMax = 25;
    public double RollOutputMin = -25;

    [Header("Position Hold PID Settings")]
    public double PositionHoldProportionalGain = 2;
    public double PositionHoldIntegralGain = 0;
    public double PositionHoldDerivativeGain = 8116;

    struct PidType
    {
        public const int
            RollAngle = 0,
            VerticalVelocity = 1,
            PitchAngle = 2,
            XPositionHold = 3,
            YPositionHold = 4,
            ZPositionHold = 5;
    }

    // Use this for initialization
    void Start()
    {
        controlInterface = transform.GetComponent<ControlInterface>();
        controlInterface.GetSensorData();

        VelocityControllers = new PidController[3];
        VelocityControllers[PidType.PitchAngle] = new PidController(PitchProportionalGain, PitchIntegralGain, PitchDerivativeGain, PitchOutputMax, PitchOutputMin);
        VelocityControllers[PidType.VerticalVelocity] = new PidController(YProportionalGain, YIntegralGain, YDerivativeGain, YOutputMax, YOutputMin);
        VelocityControllers[PidType.RollAngle] = new PidController(RollProportionalGain, RollIntegralGain, RollDerivativeGain, RollOutputMax, RollOutputMin);

        PositionControllers = new PidController[3];
        PositionControllers[0] = new PidController(PositionHoldProportionalGain, PositionHoldIntegralGain, PositionHoldDerivativeGain, PitchOutputMax, -PitchOutputMax);
        PositionControllers[1] = new PidController(PositionHoldProportionalGain, PositionHoldIntegralGain, PositionHoldDerivativeGain, YOutputMax, -YOutputMax);
        PositionControllers[2] = new PidController(PositionHoldProportionalGain, PositionHoldIntegralGain, PositionHoldDerivativeGain, RollOutputMax, -RollOutputMax);

        TicksPerUpdate = 1;
        ticksSinceLastUpdate = TicksPerUpdate;
    }

    private static ControlInterface.Instruction RollRight = new ControlInterface.Instruction
    {
        BackLeftPropellerThrottlePercentage = 1,
        FrontLeftPropellerThrottlePercentage = 1,
        BackRightPropellerThrottlePercentage = 0,
        FrontRightPropellerThrottlePercentage = 0
    };

    private static ControlInterface.Instruction RollLeft = new ControlInterface.Instruction
    {
        BackLeftPropellerThrottlePercentage = 0,
        FrontLeftPropellerThrottlePercentage = 0,
        BackRightPropellerThrottlePercentage = 1,
        FrontRightPropellerThrottlePercentage = 1
    };

    private static ControlInterface.Instruction PitchForward = new ControlInterface.Instruction
    {
        BackLeftPropellerThrottlePercentage = 1,
        FrontLeftPropellerThrottlePercentage = 0,
        BackRightPropellerThrottlePercentage = 1,
        FrontRightPropellerThrottlePercentage = 0
    };

    private static ControlInterface.Instruction PitchBackward = new ControlInterface.Instruction
    {
        BackLeftPropellerThrottlePercentage = 0,
        FrontLeftPropellerThrottlePercentage = 1,
        BackRightPropellerThrottlePercentage = 0,
        FrontRightPropellerThrottlePercentage = 1
    };

    private static ControlInterface.Instruction Up = new ControlInterface.Instruction
    {
        BackLeftPropellerThrottlePercentage = 1,
        FrontLeftPropellerThrottlePercentage = 1,
        BackRightPropellerThrottlePercentage = 1,
        FrontRightPropellerThrottlePercentage = 1
    };

    private static ControlInterface.Instruction Down = new ControlInterface.Instruction
    {
        BackLeftPropellerThrottlePercentage = 0,
        FrontLeftPropellerThrottlePercentage = 0,
        BackRightPropellerThrottlePercentage = 0,
        FrontRightPropellerThrottlePercentage = 0
    };

    ControlInterface.Instruction lastInstruction = new ControlInterface.Instruction();
    private void Update()
    {
        //Debug.Log(lastInstruction);
    }

    void FixedUpdate()
    {
        // If position hold is enabled, we simply use the corresponding PID controllers to
        // set the pitch, roll and vertical velocity targets for the other PID controllers
        if (PositionHoldEnabled)
        {
            Debug.DrawLine(transform.position, PositionHoldTarget);
            var cummulativeError = 0f;
            for (int i = 0; i < 3; i++)
            {
                PositionControllers[i].SetPoint = PositionHoldTarget[i];
                PositionControllers[i].ProcessVariable = transform.position[i];
                VelocityTargets[i] = (float)PositionControllers[i].ControlVariable(new System.TimeSpan(0, 0, 0, 0, 5));
                cummulativeError += VelocityTargets[i];
                GraphDbg.Log(string.Format("Target Hold PID Axis {0} - CurrentError", i), cummulativeError / 3, isFitX: true, logLimit: 50);
            }

        }
        else
        {
            VelocityTargets[0] = PitchAngleTarget;
            VelocityTargets[1] = VerticalVelocityTarget;
            VelocityTargets[2] = RollAngleTarget;
        }

        if (TicksPerUpdate != ticksSinceLastUpdate)
        {
            ticksSinceLastUpdate += 1;
            return;
        }

        var newInstruction = new ControlInterface.Instruction();
        var droneRigidBody = transform.GetComponent<Rigidbody>();

        //Apply height control first
        if (transform.position[PidType.VerticalVelocity] < 0)
        {
            VelocityControllers[PidType.VerticalVelocity].SetPoint = VelocityTargets[PidType.VerticalVelocity] + Mathf.Abs(droneRigidBody.velocity.y);
            VelocityControllers[PidType.VerticalVelocity].ProcessVariable = 0;
        }
        else if (transform.position[PidType.VerticalVelocity] > 0)
        {
            VelocityControllers[PidType.VerticalVelocity].SetPoint = VelocityTargets[PidType.VerticalVelocity];
            VelocityControllers[PidType.VerticalVelocity].ProcessVariable = droneRigidBody.velocity.y;
        }

        var output = VelocityControllers[PidType.VerticalVelocity].ControlVariable(new System.TimeSpan(0, 0, 0, 0, 50));
        var outputMagnitude = 0f;

        GraphDbg.Log("Vertical PID - CurrentError", droneRigidBody.velocity.y, isFitX: true, logLimit: 50);
        GraphDbg.Log("Vertical PID - IntegralTerm", (float)VelocityControllers[PidType.VerticalVelocity].IntegralTerm, isFitX: true, logLimit: 50);
        GraphDbg.Log("Vertical PID - ControlVariable", (float)output, isFitX: true, logLimit: 50);
        GraphDbg.Log("Vertical PID - ProcessVariable", (float)VelocityControllers[PidType.VerticalVelocity].ProcessVariable, isFitX: true, logLimit: 50);

        outputMagnitude = (float)(output / 100); //Normalize value
        newInstruction = (Up * outputMagnitude); //Apply initial velocities


        //Now we apply pitch and roll corrections on top of the existing instruction
        
        var axises = new int[2] { PidType.PitchAngle, PidType.RollAngle };
        foreach (var pidType in axises)
        {
            if (transform.rotation.eulerAngles[pidType] < 0)
            {
                VelocityControllers[pidType].SetPoint = VelocityTargets[pidType] + Mathf.Abs(transform.rotation.eulerAngles[pidType]);
                VelocityControllers[pidType].ProcessVariable = 0;
            } else {
                VelocityControllers[pidType].SetPoint = VelocityTargets[pidType];
                var angleValue = transform.rotation.eulerAngles[pidType];
                if (transform.rotation.eulerAngles[pidType] > 180)
                {
                    angleValue -= 360f;
                } 
                VelocityControllers[pidType].ProcessVariable = angleValue;
            }

            var axisOutput = (float)VelocityControllers[pidType].ControlVariable(new System.TimeSpan(0, 0, 0, 0, 5));
            var mag = Mathf.Abs(axisOutput) / 100;

            switch (pidType)
            {
                case 2:
                    GraphDbg.Log("Roll PID - CurrentError", transform.rotation.eulerAngles[pidType], isFitX: true, logLimit: 50);
                    GraphDbg.Log("Roll PID - ControlVariable", (float)axisOutput, isFitX: true, logLimit: 50);
                    GraphDbg.Log("Roll PID - ProcessVariable", (float)VelocityControllers[pidType].ProcessVariable, isFitX: true, logLimit: 50);
                    if (axisOutput > 0)
                    {
                        var t = (RollRight * mag);

                        newInstruction = newInstruction + (RollLeft * mag);
                    }
                    else if (axisOutput < 0)
                    {
                        var t = (RollLeft * mag);
                        newInstruction = newInstruction + (RollRight * mag);
                    }

                    break;
                case 0:
                    GraphDbg.Log("Pitch PID - CurrentError", transform.rotation.eulerAngles[pidType], isFitX: true, logLimit: 50);
                    GraphDbg.Log("Pitch PID - ControlVariable", (float)axisOutput, isFitX: true, logLimit: 50);
                    GraphDbg.Log("Pitch PID - ProcessVariable", (float)VelocityControllers[pidType].ProcessVariable, isFitX: true, logLimit: 50);

                    if (axisOutput > 0)
                    {
                        var t = (RollLeft * axisOutput);

                        newInstruction = newInstruction + (PitchForward * mag);
                    }
                    else if (axisOutput < 0)
                    {
                        var t = (RollRight * axisOutput);
                        newInstruction = newInstruction + (PitchBackward * mag);
                    }

                    break;
            }
        }
        
        lastInstruction = newInstruction;
        controlInterface.SetNextInstruction(newInstruction);

        ticksSinceLastUpdate = 0;
    }
}