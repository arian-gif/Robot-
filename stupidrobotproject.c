#include "PC_FileIO.c"

void configureAllSensors();
void drive(int motor_power); //powers both drive motors with the same power
void driveBoth(int motor_power_A, int motor_power_D); //powers both motors independently

void driveDistance(int distance, bool touch)
{
    nMotorEncoder[motorA] = 0;
    const float CM_TO_DEG = 180 / (2.75 * PI);
    if(distance > 0)
    {
        drive(20);
    }
    else
    {
        drive(-20);
    }

    if(touch)
  	{
  		while((abs(nMotorEncoder[motorA]) < abs(distance * CM_TO_DEG)))
    	{}
  	}
  	else
  	{
  		while((abs(nMotorEncoder[motorA]) < abs(distance * CM_TO_DEG)) && SensorValue[S1] == 0)
    	{}
  	}
		drive(0);
}

void claw()
{
	if(nMotorEncoder[motorB]>400)
	{
		motor[motorB]=-10;
		while(nMotorEncoder[motorB]>0)
		{}
		motor[motorB]=0;
	}
	else
	{
		motor[motorB]=10;
		while(nMotorEncoder[motorB]<450)
		{}
		motor[motorB]=0;
		driveDistance(18,true);
	}
}

void lastStep()
{
		if (getGyroDegrees(S3) < 0)
    {
    	driveBoth(-5, 5);
      while (getGyroDegrees(S3) < 0 && SensorValue[S1] == 0)
      {}
		}
		else
		{
			driveBoth(5, -5);
      while (getGyroDegrees(S3) > 0 && SensorValue[S1] == 0)
      {}
		}
		drive(0);
}

void displayStep(int step)
{
    if(step == 1)
    {
    	displayString(5, "Step 1, Retrieving box");
    }
    else if(step == 2)
    {
    	displayString(5, "Step 2, Going to drop off");
    }
    else
    {
     	displayString(5, "Step 3,Going back to origin");
    }
}

void time(float time, bool complete)
{
    if(time < 45000 && complete)
    {

    	displayString(5, "Time: %.2f seconds            .",time/1000.0,);
    	displayString(6,"Good Delivery Time");
    }
    else if(time > 45000 && complete)
    {

    	displayString(5, "Time: %.2f seconds            .",time/1000.0,);
    	displayString(6,"Bad Delivery Time");
    }
    else if(!complete)
    {
    	displayString(5, "Task incomplete               .");
    }
}

void rotateRobot(float angle, bool touch)
{
    if (getGyroDegrees(S3) < angle)
    {
        driveBoth(-5, 5);
        if (touch)
        {
            while (getGyroDegrees(S3) < angle)
            {}
        }
        else
        {
            while (getGyroDegrees(S3) < angle && SensorValue[S1] == 0)
            {}
        }
    }
    else
    {
        driveBoth(5, -5);
        if (touch)
        {
            while (getGyroDegrees(S3) > angle)
            {}
        }
        else
        {
            while (getGyroDegrees(S3) > angle && SensorValue[S1] == 0)
            {}

        }
    }
    drive(0);
}

bool filterfile(int x, int y)
{
    return (x>=0&&y>0)&&(y<=4&&x<=4);
}

float angle(float x, float pre_x, float y,float pre_y,bool touch,int turn,int adj)
{
		float new_x=0,new_y=0;
    x=x*15;
    y=y*15;
    pre_x= pre_x*15;
    pre_y=pre_y*15;
		new_x= x-pre_x;
		new_y= y-pre_y;
    float degrees = atan2(new_x,new_y) * 180 / PI;
    if(turn ==3)
    {
    	degrees= degrees-5;
    }
    degrees +=adj;
    rotateRobot(degrees, touch);
    return degrees;
}

float shortestD(float x, float pre_x, float y,float pre_y,bool touch)
{
    float new_x=0,new_y=0;
    x=x*15;
    y=y*15;
    pre_x= pre_x*15;
    pre_y=pre_y*15;
		new_x= x-pre_x;
		new_y= y-pre_y;

    float shortest_distance = sqrt(new_x * new_x + new_y * new_y);
    driveDistance(shortest_distance, touch);
    return shortest_distance;

}
int errorChecker(float distance,float angle, int attempt)
{
	if(attempt ==1)
	{
		driveDistance(-distance, false);
		rotateRobot(-angle,false);
		return 5;
	}
	else if(attempt ==2)
	{
		driveDistance(-distance, false);
		rotateRobot(-angle,false);
		return -10;
	}
	return 0;
}
bool colour(int pre_x,int pre_y,int adj)
{
		int final_x =0,final_y=0;
		if (SensorValue(S2) == (int)colorRed)
        {
            final_x = 0;
            final_y = 5;
            angle(final_x, pre_x, final_y,pre_y, true,2,adj);
            shortestD(final_x,pre_x, final_y,pre_y, true);
            return true;
        }
    else if (SensorValue(S2) != (int)colorRed)
        {
            final_x = 4;
            final_y = 5;
            angle(final_x, pre_x, final_y,pre_y, true,2,0);
            shortestD(final_x,pre_x, final_y,pre_y, true);
            return false;
        }
       return false;
}

task main()
{
		configureAllSensors();

    TFileHandle fin;
    bool fileOkay = openReadPC(fin, "file.txt");
    if(!fileOkay)
    {
    displayString(5, "Error");
    wait1Msec(5000);
    }

    else {
    for(int delivery =1;delivery <=3;delivery++)
    {
    	time1[T1]=0;
    	resetGyro(S3);
    	wait1Msec(50);
    	float x_coordinate=0, y_coordinate=0;
    	readFloatPC(fin, x_coordinate);
    	readFloatPC(fin, y_coordinate);


    	int attempts = 0;
    	bool running = true;
    	float turn =0,distance=0;
    	if(filterfile(x_coordinate, y_coordinate))
    	{
    	displayStep(1);
			int adjustment =0;
   		while (running && attempts < 3)
    		{

        turn = angle( x_coordinate, 2,y_coordinate,0, false,1,adjustment);
       	distance = shortestD( x_coordinate,2,y_coordinate,0, false);
        if (SensorValue[S1] == 0)
        {
        		attempts++;
            adjustment = errorChecker(distance,turn,attempts);
        }
        else if(SensorValue[S1]==1)
        {
        running =!running;
        }
    	}
    	if (attempts < 3)
    	{
    	nMotorEncoder[motorB]=0;
      claw();
        displayStep(2);
        int final_x=0,final_y=0;
        if(colour(x_coordinate,y_coordinate,adjustment))
        {
        	final_x =0;
        	final_y=5;
        }
        else
        {
        	final_x =5;
        	final_y=5;
        }
        claw();

        //drive back to start
        displayStep(3);
        angle(2,final_x, 0,final_y, true,3,0);
        shortestD(2, final_x, 0, final_y, true);
        lastStep();
        float finish_time = (time1[T1]);
    		time(finish_time,true);

    }
    else
    	{
    		turn = turn +10;
    		driveDistance(-distance,false);
    		rotateRobot(-turn,false);
    		time(0,false);
    	}
		}
		else
			{
				displayString(5,"Not on Grid");
				wait1Msec(5000);
		}
    wait1Msec(10000);
    displayString(5,"                                 .");
    displayString(6,"                                 .");
  	}
  }


}

void configureAllSensors()
{
    SensorType[S1] = sensorEV3_Touch;
    SensorType[S2] = sensorEV3_Color;
    wait1Msec(50);
    SensorMode[S2] = modeEV3Color_Color;
    wait1Msec(50);
    SensorType[S3] = sensorEV3_Gyro;
    wait1Msec(50);
    SensorMode[S3] = modeEV3Gyro_Calibration;
    wait1Msec(100);
    SensorMode[S3] = modeEV3Gyro_RateAndAngle;
    wait1Msec(50);
}

void drive(int motor_power)
{
    motor[motorA] = motor[motorD] = motor_power;
}

void driveBoth(int motor_power_A, int motor_power_D)
{
    motor[motorA] = motor_power_A;
    motor[motorD] = motor_power_D;
}
