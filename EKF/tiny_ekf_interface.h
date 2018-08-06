/* SensorFusion: Sensor fusion on GPS, Baro, and Rangefinder in PX4 using TinyEKF.  
 *
 * Copyright (C) 2015 Simon D. Levy
 *
 * MIT License
 */


// These must be defined before including TinyEKF.h
#define Nsta 1     // One state value: Height
#define Mobs 2     // Three measurements: GPS, Barometer, and Rangefinder


#include <TinyEKF/src/TinyEKF.h>

class AltitudeFusionTinyEKF : public TinyEKF {

    public:

        AltitudeFusionTinyEKF()
        {            
            // We approximate the process noise using a small constant
            this->setQ(0, 0, .0001);

            // Same for measurement noise
            this->setR(0, 0, 1.5f);
            this->setR(1, 1, 0.25f);
            //this->setR(2, 2, 0.5f);
        }

    protected:

        void model(double fx[Nsta], double F[Nsta][Nsta], double hx[Mobs], double H[Mobs][Nsta])
        {
            // Process model is f(x) = x
            fx[0] = this->x[0];

            // So process model Jacobian is identity matrix
            F[0][0] = 1;

            // Measurement function simplifies the relationship between state and sensor readings for convenience.
            // A more realistic measurement function would distinguish between state value and measured value; e.g.:
            //   hx[0] = pow(this->x[0], 1.03);
            //   hx[1] = 1.005 * this->x[1];
            //   hx[2] = .9987 * this->x[1] + .001;
            hx[0] = this->x[0]; // GPS
            hx[1] = this->x[0]; // Baro 
            //hx[2] = this->x[0]; // Rangefinder

            // Jacobian of measurement function
            H[0][0] = 1;        // GPS
            H[1][0] = 1 ;       // Baro
            //H[2][0] = 1 ;       // Rangefinder
        }
};