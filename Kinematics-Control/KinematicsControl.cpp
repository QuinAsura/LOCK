/*
 *
 *  Created on: 01.03.2015
 *      Authors: Sibi Sankar, Sanjay Shreedharan, Adithya S   
 *
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <iostream>
#include <stdio.h>
#include <cstdlib>

class Bot
{
    public:
        double Wradius,Blength,Bvelocity,pwm_lower,pwm_upper,omega_lower,omega_upper,rpm_lower,rpm_upper;
        int Wvelocities[2];
        double Kp,Ki,Kd,error,prev_error,Integ;
        long int time,prev_time,freq;

        Bot(double p,double i,double d)
        {
            Kp=p;Ki=i;Kd=d;error=0;prev_error=0;Integ=0;
            time=0;prev_time=0;freq = getTickFrequency();
            Wradius = 0.037;
            Blength = 0.106;
            Bvelocity = 0.065;
            pwm_lower = 45.0;
            pwm_upper = 240.0;
            omega_lower = 0.0;
            omega_upper = 8.35;

        }

        void Uni2DiffKinematics(double w)
        {
            double sum  = 2*Bvelocity/(Wradius);
            double diff = Blength*w/(Wradius);
            double l_w = (sum+diff)/2;
            double r_w = (sum-diff)/2;
            if ((l_w > omega_upper) || (r_w > omega_upper))
            {
                if (l_w > omega_upper)
                {
                    double temp = l_w - omega_upper;
                    l_w = omega_upper;
                    r_w -= temp;
                    if(r_w < omega_lower)
                    {
                        r_w =omega_lower;
                    }

                }
                else
                {
                    double temp = r_w -omega_upper;
                    r_w = omega_upper;
                    l_w -= temp;
                    if(l_w < omega_lower)
                    {
                        l_w=omega_lower;
                    }
                }
            }
            else if ((l_w < omega_lower) || (r_w < omega_lower))
            {
                if(l_w < omega_lower)
                {
                    double temp = omega_lower - l_w;
                    l_w = omega_lower;
                    r_w += temp;
                    if (r_w > omega_upper)
                    {r_w = omega_upper;}
                }
                else
                {
                    double temp = omega_lower - r_w;
                    r_w = omega_lower;
                    l_w += temp;
                    if(l_w > omega_upper)
                    {
                        l_w = omega_upper;
                    }
                }
            }
            l_w = pwm_lower + (((pwm_upper - pwm_lower)/(omega_upper -omega_lower))*l_w);
            r_w = pwm_lower + (((pwm_upper - pwm_lower)/(omega_upper -omega_lower))*r_w);
            Wvelocities[0]=int(l_w);
            Wvelocities[1]=int(r_w);
            //cout<<"Left_wheel:"<<l_w<<"\t"<<"Right_wheel:"<<r_w<<"\t"<<"PID_Value"<<w<<endl;
        }

        void ResetController()
        {
            prev_time = getTickCount();
            error=0;prev_error=0;Integ=0;
        }

        double PID_Controller(double accel)
        {
            double signal;
            error = accel;
            Integ+=error;
            time = getTickCount();
            double dt = (double(time - prev_time))/freq;
            signal = (error*Kp) + ((error - prev_error)*Kd*dt) + (Integ*Ki/dt);
            prev_error = error;
            prev_time = time;
            return signal;
        }
};

