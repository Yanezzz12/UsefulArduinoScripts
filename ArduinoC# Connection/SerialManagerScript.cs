using System;
using UniityEngine;
using System.IO.Ports;
using System.Threading;
//Pasa este y el de arduino a github
public class SerialManagerScript : MonoBehaviour 
{
    public String COM;
    public delegate void SerialEvent(string incomingString);
    public static event SerialEvent WhenReceiveDataCall;

    //Private variables
    private bool abort;
    private Thread serialThread;
    private static SerialPort serialPort;
    private SynchronizationContext mainThread;

    private char incomingChar;
    private String incomingString;

void Start() 
{
    serialPort = new SerialPort(COM, 9600);
    serialPort.Open();
    serialPort.DiscardInBuffer();
    serialPort.DiscardOutBuffer();
    serialPort.ReadTimeout = 100; //Línea muy importante
    mainThread = SynchronizationContext.Current;

    if(mainThread == null)
    {
        mainThread = new SynchronizationContext();
    }

    serialThread = new Thread();

    if(serialPort.IsOpen)
        serialThread.Start();
}

private void Receive() //Hilo secundario
{
    while(true)
    {
        if(abort)
        {
            serialThread.Abort();
            break;
        }
    
        try
        {
            incomingChar = (char)serialPort.ReadChar();
        }
        catch(Exception e)
        {
            print("Error SerialManagerScript Receive()");
        }
    
        if(!incomingChar.Equals('\n'))
        {
            incomingString += incomingChar;
        }
        else
        {
            mainThread.Send((object state) => 
            {
                if(WhenReceiveDataCall != null)
                    WhenReceiveDataCall(incomingString);
            }, null);
        }
    }
}

public static void SendInformation(string infoSend)
{
    serialPort.Write(infoToSend);
}

private void OnApplicationQuit() 
{
    abort = true;
    serialPort.DiscardInBuffer();
    serialPort.DiscardOutBuffer();
    serialPort.Close();
}


}