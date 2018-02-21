using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class KeyboardController : MonoBehaviour {

    private ControlInterface controlInterface;

	// Use this for initialization
	void Start () {
        controlInterface = transform.Find("Body").GetComponent<ControlInterface>();
        controlInterface.GetSensorData();
	}
	
	void Update () {
        var newInstruction = new ControlInterface.Instruction();

        if (Input.GetKey("space"))
        {
            // Add 20% thrust to all propellers if space is pressed
            newInstruction.BackLeftPropellerThrottlePercentage = newInstruction.BackLeftPropellerThrottlePercentage + 0.9f;
            newInstruction.BackRightPropellerThrottlePercentage = newInstruction.BackRightPropellerThrottlePercentage + 0.8f;
            newInstruction.FrontLeftPropellerThrottlePercentage = newInstruction.FrontLeftPropellerThrottlePercentage + 0.9f;
            newInstruction.FrontRightPropellerThrottlePercentage = newInstruction.FrontRightPropellerThrottlePercentage + 0.8f;
        }

        // Queue instruction for execution in controller

        controlInterface.SetNextInstruction(newInstruction);
    }
}
