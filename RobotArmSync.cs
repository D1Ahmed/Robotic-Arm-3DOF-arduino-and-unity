using UnityEngine;
using System.IO.Ports;

public class RobotArmSync : MonoBehaviour
{
    SerialPort sp = new SerialPort("COM3", 9600);

    public Transform baseJoint;
    public Transform shoulderJoint;
    public Transform elbowJoint;

    bool manualMode = false;

    void Start()
    {
        try { sp.Open(); sp.ReadTimeout = 10; }
        catch (System.Exception e) { Debug.LogError(e.Message); }
    }

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.M))
        {
            manualMode = !manualMode;
            Debug.Log("Manual Mode: " + manualMode);
        }

        //ONLY follow Arduino if we are not in Manual Mode
        if (sp.IsOpen && !manualMode)
        {
            try
            {
                string data = sp.ReadLine();
                string[] angles = data.Split(',');

                if (angles.Length >= 3)
                {
                    float a1 = 180 - float.Parse(angles[0]);
                    float a2 = 180 - float.Parse(angles[1]);
                    float a3 = float.Parse(angles[2]);
                    baseJoint.localEulerAngles = new Vector3(0, a1, 0);
                    shoulderJoint.localEulerAngles = new Vector3(a2, 0, 0);
                    elbowJoint.localEulerAngles = new Vector3(a3, 0, 0);
                }
            }
            catch { }
        }
        if (Input.GetKeyDown(KeyCode.S))
        {
            int b = (int)baseJoint.localEulerAngles.y;
            int s = (int)shoulderJoint.localEulerAngles.x;
            int e = (int)elbowJoint.localEulerAngles.x;

            string command = b + "," + s + ".," + e + "\n";
            sp.Write(command);
            Debug.Log("send to physcial arm : " + command);
        }
    }
}