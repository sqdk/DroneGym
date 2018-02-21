using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ControlInterface : MonoBehaviour {

    public class SensorData
    {
        public Quaternion BodyRotation;
        public Vector3 FrontRightPropellerPosition;
        public Vector3 FrontLeftPropellerPosition;
        public Vector3 BackRightPropellerPosition;
        public Vector3 BackLeftPropellerPosition;
    }

    public class Instruction
    {
        public float FrontRightPropellerThrottlePercentage;
        public float FrontLeftPropellerThrottlePercentage;
        public float BackRightPropellerThrottlePercentage;
        public float BackLeftPropellerThrottlePercentage;

        public override string ToString()
        {
            return string.Format("{0}%     {1}%\n FL \\   / FR\n      X\n BL /  \\  BR\n{2}%     {3}%",
                FrontLeftPropellerThrottlePercentage,
                FrontRightPropellerThrottlePercentage,
                BackLeftPropellerThrottlePercentage,
                BackRightPropellerThrottlePercentage);
        }
        /*
        public Instruction Normalize()
        {
            var newInstruction = new Instruction();
            //Pull everything out
            var values = new float[4];
            values[0] = this.FrontLeftPropellerThrottlePercentage;
            values[1] = this.FrontRightPropellerThrottlePercentage;
            values[2] = this.BackLeftPropellerThrottlePercentage;
            values[3] = this.BackRightPropellerThrottlePercentage;

            var max = Mathf.Max(values);
            var min = Mathf.Min(values);

            newInstruction.FrontLeftPropellerThrottlePercentage = (values[0] - min) / (max - min);
            newInstruction.FrontRightPropellerThrottlePercentage = (values[1] - min) / (max - min);
            newInstruction.BackLeftPropellerThrottlePercentage = (values[2] - min) / (max - min);
            newInstruction.BackRightPropellerThrottlePercentage = (values[3] - min) / (max - min);

            return newInstruction;
        }*/

        public static Instruction operator +(Instruction i1, Instruction i2)
        {
            var averagedInstruction = new Instruction();
            averagedInstruction.BackLeftPropellerThrottlePercentage = (i1.BackLeftPropellerThrottlePercentage + i2.BackLeftPropellerThrottlePercentage);
            averagedInstruction.BackRightPropellerThrottlePercentage = (i1.BackRightPropellerThrottlePercentage + i2.BackRightPropellerThrottlePercentage);
            averagedInstruction.FrontLeftPropellerThrottlePercentage = (i1.FrontLeftPropellerThrottlePercentage + i2.FrontLeftPropellerThrottlePercentage);
            averagedInstruction.FrontRightPropellerThrottlePercentage = (i1.FrontRightPropellerThrottlePercentage + i2.FrontRightPropellerThrottlePercentage);

            return averagedInstruction;
        }

        public static Instruction operator *(Instruction i, float d)
        {
            var newInstruction = new Instruction();
            newInstruction.BackLeftPropellerThrottlePercentage = i.BackLeftPropellerThrottlePercentage * d;
            newInstruction.FrontRightPropellerThrottlePercentage = i.FrontRightPropellerThrottlePercentage * d;
            newInstruction.BackRightPropellerThrottlePercentage = i.BackRightPropellerThrottlePercentage * d;
            newInstruction.FrontLeftPropellerThrottlePercentage = i.FrontLeftPropellerThrottlePercentage * d;

            return newInstruction;
        }
    }

    public float PropellerForce = 3000;
    Instruction currentInstruction = new Instruction();

    // Use this for initialization
    void Start () {
    }

    void Update()
    {
        ExecuteInstruction(currentInstruction);
    }

    private void ExecuteInstruction(Instruction instruction)
    {
        var sensorData = GetSensorData();
        Debug.DrawLine(transform.position, transform.position+new Vector3(0,0.1f,0), Color.green, 100000);

        //instruction = instruction.Normalize();

        Rigidbody droneRigidBody = transform.GetComponent<Rigidbody>();
        Quaternion droneRotation = transform.rotation;
        // Apply force accordingly to each propeller location
        var frontLeftForceVector = droneRotation * new Vector3(0f, instruction.FrontLeftPropellerThrottlePercentage * PropellerForce, 0f);
        var frontRightForceVector = droneRotation * new Vector3(0f, instruction.FrontRightPropellerThrottlePercentage * PropellerForce, 0f);
        var backLeftForceVector = droneRotation * new Vector3(0f, instruction.BackLeftPropellerThrottlePercentage * PropellerForce, 0f);
        var backRightForceVector = droneRotation * new Vector3(0f, instruction.BackRightPropellerThrottlePercentage * PropellerForce, 0f);

        droneRigidBody.AddForceAtPosition(frontLeftForceVector, sensorData.FrontLeftPropellerPosition);
        droneRigidBody.AddForceAtPosition(frontRightForceVector, sensorData.FrontRightPropellerPosition);
        droneRigidBody.AddForceAtPosition(backLeftForceVector, sensorData.BackLeftPropellerPosition);
        droneRigidBody.AddForceAtPosition(backRightForceVector, sensorData.BackRightPropellerPosition);

        Debug.DrawLine(sensorData.FrontLeftPropellerPosition, sensorData.FrontLeftPropellerPosition+frontLeftForceVector.normalized, Color.red);
        Debug.DrawLine(sensorData.FrontRightPropellerPosition, sensorData.FrontRightPropellerPosition+frontRightForceVector.normalized, Color.red);
        Debug.DrawLine(sensorData.BackLeftPropellerPosition, sensorData.BackLeftPropellerPosition+backLeftForceVector.normalized, Color.red);
        Debug.DrawLine(sensorData.BackRightPropellerPosition, sensorData.BackRightPropellerPosition+backRightForceVector.normalized, Color.red);
    }

    public void SetNextInstruction(Instruction instruction)
    {
        currentInstruction = instruction;
    }

    // Update is called once per frame
    public SensorData GetSensorData() {
        SensorData newReading = new SensorData
        {
            BodyRotation = transform.transform.rotation,
            // Get propeller positions by name
            FrontLeftPropellerPosition = GameObject.FindGameObjectWithTag("PropellerFrontLeft").transform.position,
            FrontRightPropellerPosition = GameObject.FindGameObjectWithTag("PropellerFrontRight").transform.position,
            BackLeftPropellerPosition = GameObject.FindGameObjectWithTag("PropellerBackLeft").transform.position,
            BackRightPropellerPosition = GameObject.FindGameObjectWithTag("PropellerBackRight").transform.position
        };

        return newReading;
    }
}
