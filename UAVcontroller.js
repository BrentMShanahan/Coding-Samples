/**
 * Unity 3D quad rotor drone autonomous flight script
 *
 * @Author Brent Shanahan.
 */

#pragma strict
import System.Collections.Generic; 

private var UAV : GameObject;
private var LR : GameObject;
private var RR : GameObject;
private var LF : GameObject;
private var RF : GameObject;
private var boxes : GameObject;
private var truck : GameObject;
public var gimbal : Camera;
public var cameras : Camera[];

private var pathpoints : GameObject;
private var pathpoint : Vector3;
private var pathArray = [Vector3(0,120,0),Vector3(0,120,100), Vector3(-23, 120, 129.4), Vector3(-57.4, 120, 138.8), Vector3(-105.3, 120, 121.7), Vector3(-124.8, 120, 55.5), Vector3(-84.5, 120, 29.2), Vector3(-36.85, 120, 16.61)];
private var prevppNum : int = 0;
private var ppNum : int = 0;
private var ppTime : float = 0;
private var snap : int = 1;
private var tick : int = 1;
private var fly : boolean = false;

private var ua : Vector3;
private var lastTime : float;
private var errSum : float[,] = new float[7,2];
private var lastErr : float[,] = new float[7,2];

private var socketObject : GameObject;
private var socket : SocketIO.SocketIOComponent;

function Start() {
	UAV = GameObject.Find("UAV");
	LR = GameObject.Find("UAV/prop_LR");
	RR = GameObject.Find("UAV/prop_RR");
	LF = GameObject.Find("UAV/prop_LF");
	RF = GameObject.Find("UAV/prop_RF");
	boxes = GameObject.Find("Packages");
	truck = GameObject.Find("PostalTruck");

	pathpoints = GameObject.Find("Path");
	pathpoint = pathArray[ppNum];
	boxes.SetActive(false);
	truck.SetActive(false);
	
	socketObject = GameObject.Find("SocketIO");
	socket = socketObject.GetComponent(SocketIO.SocketIOComponent);
	socket.On("open", socketOpen);
	socket.On("MMSsent", MMSsent);
}

function FixedUpdate () {
	if(fly){
		var time : float = Time.time;
		var timeChange : float = time - lastTime;
		var prevPathPoint : Vector3 = pathArray[prevppNum];
		pathpoint = pathArray[ppNum];
		var uavPos = UAV.transform.position;
		var uavAn = UAV.transform.localEulerAngles;

		var fixX : float = fixAxis(uavAn.x);
		var fixY : float = uavAn.y;
		var fixZ : float = fixAxis(uavAn.z);
		ua = Vector3(fixX, fixY, fixZ);
		
		var uavDir : Vector3 = UAV.transform.forward;
		var tdist : Vector3 = pathpoint - uavPos;
		var dist : float = Vector3.Distance(pathpoint, uavPos);
		var ppdist : float = Vector3.Distance(prevPathPoint, pathpoint);

		if (tdist.x < 8 && tdist.x > -8 && tdist.y < 2 && tdist.y > -2 && tdist.z < 8 && tdist.z > -8){
			prevppNum = ppNum;
			ppNum += 1;
			ppTime = 0;
			if (ppNum == pathArray.Length){
				prevppNum = ppNum - 1;
				ppNum = 0;
				ppTime = 0;
			}
			tick = 1;
		}
		
		var zVel : float = UAV.transform.InverseTransformDirection(GetComponent.<Rigidbody>().velocity).z;
		var xVel : float = UAV.transform.InverseTransformDirection(GetComponent.<Rigidbody>().velocity).x;
		
		var ease : float;
		if (ppTime < (300 * Time.deltaTime)){
			ease = Ease(ppTime, 0, 1, (300 * Time.deltaTime));
		}else{ease = 1;}
		
		var distErr : float = Error(0, 0, dist, timeChange, 1, 0, 0);
		var velErr : float = Error(1, zVel, distErr, timeChange, 0.2, 0, 1);
		
		var yaw = Mathf.Atan2(Vector3.Dot(UAV.transform.up, Vector3.Cross(uavDir, tdist)),Vector3.Dot(uavDir, tdist)) * Mathf.Rad2Deg; 
		
		var ppVect : Vector3 = pathpoint - prevPathPoint;
		var roll = Mathf.Atan2(Vector3.Dot(Vector3(0,1,0), Vector3.Cross(ppVect, tdist)),Vector3.Dot(ppVect, tdist)) * Mathf.Rad2Deg; 
		var rollErr = Error(2, 0, roll, timeChange, 1, 0, .5);
		
		var pitchOutput : float = Error(3, ua.x, velErr, timeChange, 1, 0, 0.2);
	    var yawOutput : float = Error(4, 0, yaw, timeChange, 2, 0, 1);
	    var rollOutput : float = Error(5, ua.z, -rollErr, timeChange, 1, 0, 0.2);
	    
		var output : Vector3 = Vector3(pitchOutput * ease, yawOutput, rollOutput * ease);
		
		var thrustErr : float = Error(6, uavPos.y, pathpoint.y, timeChange, 3, 0, 7);
		var thrust : float = 122 + thrustErr;
		
		var stabilLR : float;
		var stabilRR : float;
		var stabilLF : float;
		var stabilRF : float;
		var stabilYawC : float;
		var stabilYawCC : float;
		var forceX : float = output.x;
		var forceY: float;
		if (tdist.x < 8 && tdist.x > -8 && tdist.z < 8 && tdist.z > -8){
			forceY = 0;
		}else{forceY = output.y;}
		var forceZ : float = output.z;
		
		stabilLR = thrust - forceX + forceZ;
		stabilRR = thrust - forceX - forceZ;
		stabilLF = thrust + forceX + forceZ;
		stabilRF = thrust + forceX - forceZ;
		
		UAV.GetComponent.<Rigidbody>().AddForceAtPosition(UAV.transform.up * stabilLR * Time.deltaTime, LR.transform.position, ForceMode.Force);
		UAV.GetComponent.<Rigidbody>().AddForceAtPosition(UAV.transform.up * stabilRR * Time.deltaTime, RR.transform.position, ForceMode.Force);
		UAV.GetComponent.<Rigidbody>().AddForceAtPosition(UAV.transform.up * stabilLF * Time.deltaTime, LF.transform.position, ForceMode.Force);
		UAV.GetComponent.<Rigidbody>().AddForceAtPosition(UAV.transform.up * stabilRF * Time.deltaTime, RF.transform.position, ForceMode.Force);
		
		LR.transform.Rotate(0, -(stabilLR * 15 * Time.deltaTime), 0);
		RR.transform.Rotate(0, stabilRR * 15 * Time.deltaTime, 0);
		LF.transform.Rotate(0, stabilLF * 15 * Time.deltaTime, 0);
		RF.transform.Rotate(0, -(stabilRF * 15 * Time.deltaTime), 0);
		
		stabilYawC = forceY;
		stabilYawCC = -(forceY);
		
		if (ua.y < 0){
			UAV.GetComponent.<Rigidbody>().AddForceAtPosition(UAV.transform.forward * stabilYawCC * Time.deltaTime, LR.transform.position, ForceMode.Force);
			UAV.GetComponent.<Rigidbody>().AddForceAtPosition((UAV.transform.forward * -1) * stabilYawCC * Time.deltaTime, RF.transform.position, ForceMode.Force);
		}else if (ua.y > 0){
			UAV.GetComponent.<Rigidbody>().AddForceAtPosition(UAV.transform.forward * stabilYawC * Time.deltaTime, RR.transform.position, ForceMode.Force);
			UAV.GetComponent.<Rigidbody>().AddForceAtPosition((UAV.transform.forward * -1) * stabilYawC * Time.deltaTime, LF.transform.position, ForceMode.Force);
		}
		ppTime += Time.deltaTime;
		lastTime = time;
		
		if (ppNum == 7){
			boxes.SetActive(true);
			truck.SetActive(true);
			snap = 0;
		}
		if (ppNum == 3 && snap == 0){
			Application.CaptureScreenshot("C:/Users/Brent/Documents/GitHub/sendSMS/public/images/UnityScreenshot.png");
			socket.Emit("UnityMMS");
			Debug.Log("Message Sent");
			snap = 1;
		}
		if (ppNum == 2 || ppNum == 3 || ppNum == 4){
			gimbal.enabled = true;
			if (tick == 1){
				for (cam in cameras){
					cam.enabled = false;
				}
				tick = 0;
				//Debug.Log("tick");
			}
		}else{
			gimbal.enabled = false;
			closestCam();
		}
	}
}

function fixAxis(axis : float){
	var fixA : float;
	if (axis > 180){
		fixA = -(360 - Mathf.Abs(axis));
	}else{
		fixA = axis;
	}
	return fixA;
}
function Ease(t : float, b : float, c : float, d : float){
	return c*(t/d)+b;
}
function Error(i : int, input : float, setPoint : float, tc : float, kp : float, ki : float, kd : float){
	var error : float = setPoint - input;
	if (float.IsNaN(error)){error = 0;}
	errSum[i,0] += (error * tc);
	var dErr : float = (error - lastErr[i,1]) / tc;
	var Output : float = (kp * error) + (ki * errSum[i,0]) + (kd * dErr);
	lastErr[i,1] = error;
	return Output;
}
function OnGUI () {
	//GUI.color = Color.black;
	if(GUI.Button(Rect(10,10,100,30), "Fly")){
		if (!fly){
			fly = true;
		}else{
			fly = false;
		}
	}
}
function socketOpen(e : SocketIO.SocketIOEvent){
	Debug.Log("[SocketIO] Open received: " + e.name + " " + e.data);
}
function MMSsent(){
	Debug.Log("[SOCKETIO] MMS MESSAGE SENT SUCCESSFULLY");
}
function closestCam() : Camera{
	var closest : Camera;
	var distance = Mathf.Infinity; 
	var position = transform.position; 	
	for (var cam : Camera in cameras)  { 
		var diff = (cam.transform.position - position);
		var curDistance = diff.sqrMagnitude; 
		if (curDistance < distance) { 
			closest = cam; 
			distance = curDistance; 
		} 
		cam.enabled = false;
	} 
	closest.enabled = true;
}
