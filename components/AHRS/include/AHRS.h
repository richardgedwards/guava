// #define sampleFreq	512.0f		// sample frequency in Hz
// #define betaDef		0.1f		// 2 * proportional gain
// volatile float beta = betaDef;								// 2 * proportional gain (Kp)
// volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	// quaternion of sensor frame relative to auxiliary frame

class AHRS 
{
    protected:
        float _beta, _sampleFreq, a, b, c, d;
        
    public:
        AHRS(float sampleFreq=483.0f, float beta=40.0f)
            : _beta(beta), _sampleFreq(sampleFreq), a(1.0f), b(0.0f), c(0.0f), d(0.0f)  {

        }

        void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
            float recipNorm;
            float s0, s1, s2, s3;
            float qDot1, qDot2, qDot3, qDot4;
            float hx, hy;
            float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

            // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
            if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
                MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);
                return;
            }

            // Rate of change of quaternion from gyroscope
            qDot1 = 0.5f * (-b * gx - c * gy - d * gz);
            qDot2 = 0.5f * ( a * gx + c * gz - d * gy);
            qDot3 = 0.5f * ( a * gy - b * gz + d * gx);
            qDot4 = 0.5f * ( a * gz + b * gy - c * gx);

            // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
            if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

                // Normalise accelerometer measurement
                recipNorm = invSqrt(ax * ax + ay * ay + az * az);
                ax *= recipNorm;
                ay *= recipNorm;
                az *= recipNorm;   

                // Normalise magnetometer measurement
                recipNorm = invSqrt(mx * mx + my * my + mz * mz);
                mx *= recipNorm;
                my *= recipNorm;
                mz *= recipNorm;

                // Auxiliary variables to avoid repeated arithmetic
                _2q0mx = 2.0f * a * mx;
                _2q0my = 2.0f * a * my;
                _2q0mz = 2.0f * a * mz;
                _2q1mx = 2.0f * b * mx;
                _2q0 = 2.0f * a;
                _2q1 = 2.0f * b;
                _2q2 = 2.0f * c;
                _2q3 = 2.0f * d;
                _2q0q2 = 2.0f * a * c;
                _2q2q3 = 2.0f * c * d;
                q0q0 = a * a;
                q0q1 = a * b;
                q0q2 = a * c;
                q0q3 = a * d;
                q1q1 = b * b;
                q1q2 = b * c;
                q1q3 = b * d;
                q2q2 = c * c;
                q2q3 = c * d;
                q3q3 = d * d;

                // Reference direction of Earth's magnetic field
                hx = mx * q0q0 - _2q0my * d + _2q0mz * c + mx * q1q1 + _2q1 * my * c + _2q1 * mz * d - mx * q2q2 - mx * q3q3;
                hy = _2q0mx * d + my * q0q0 - _2q0mz * b + _2q1mx * c - my * q1q1 + my * q2q2 + _2q2 * mz * d - my * q3q3;
                _2bx = sqrt(hx * hx + hy * hy);
                _2bz = -_2q0mx * c + _2q0my * b + mz * q0q0 + _2q1mx * d - mz * q1q1 + _2q2 * my * d - mz * q2q2 + mz * q3q3;
                _4bx = 2.0f * _2bx;
                _4bz = 2.0f * _2bz;

                // Gradient decent algorithm corrective step
                s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * c * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * d + _2bz * b) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * c * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
                s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * b * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * d * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * c + _2bz * a) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * d - _4bz * b) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
                s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * c * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * c - _2bz * a) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * b + _2bz * d) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * a - _4bz * c) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
                s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * d + _2bz * b) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * a + _2bz * c) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * b * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
                recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
                s0 *= recipNorm;
                s1 *= recipNorm;
                s2 *= recipNorm;
                s3 *= recipNorm;

                // Apply feedback step
                qDot1 -= _beta * s0;
                qDot2 -= _beta * s1;
                qDot3 -= _beta * s2;
                qDot4 -= _beta * s3;
            }

            // Integrate rate of change of quaternion to yield quaternion
            a += qDot1 * (1.0f / _sampleFreq);
            b += qDot2 * (1.0f / _sampleFreq);
            c += qDot3 * (1.0f / _sampleFreq);
            d += qDot4 * (1.0f / _sampleFreq);

            // Normalise quaternion
            recipNorm = invSqrt(a * a + b * b + c * c + d * d);
            a *= recipNorm;
            b *= recipNorm;
            c *= recipNorm;
            d *= recipNorm;
        }


        void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az){
            float recipNorm;
            float s0, s1, s2, s3;
            float qDot1, qDot2, qDot3, qDot4;
            float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

            // Rate of change of quaternion from gyroscope
            qDot1 = 0.5f * (-b * gx - c * gy - d * gz);
            qDot2 = 0.5f * ( a * gx + c * gz - d * gy);
            qDot3 = 0.5f * ( a * gy - b * gz + d * gx);
            qDot4 = 0.5f * ( a * gz + b * gy - c * gx);

            // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
            if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

                // Normalise accelerometer measurement
                recipNorm = invSqrt(ax * ax + ay * ay + az * az);

                ax *= recipNorm;
                ay *= recipNorm;
                az *= recipNorm;   

                // Auxiliary variables to avoid repeated arithmetic
                _2q0 = 2.0f * a;
                _2q1 = 2.0f * b;
                _2q2 = 2.0f * c;
                _2q3 = 2.0f * d;
                _4q0 = 4.0f * a;
                _4q1 = 4.0f * b;
                _4q2 = 4.0f * c;
                _8q1 = 8.0f * b;
                _8q2 = 8.0f * c;
                q0q0 = a * a;
                q1q1 = b * b;
                q2q2 = c * c;
                q3q3 = d * d;

                // Gradient decent algorithm corrective step
                s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
                s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * b - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
                s2 = 4.0f * q0q0 * c + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
                s3 = 4.0f * q1q1 * d - _2q1 * ax + 4.0f * q2q2 * d - _2q2 * ay;
                recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
                s0 *= recipNorm;
                s1 *= recipNorm;
                s2 *= recipNorm;
                s3 *= recipNorm;

                // Apply feedback step
                qDot1 -= _beta * s0;
                qDot2 -= _beta * s1;
                qDot3 -= _beta * s2;
                qDot4 -= _beta * s3;
            }

            // Integrate rate of change of quaternion to yield quaternion
            a += qDot1 * (1.0f / _sampleFreq);
            b += qDot2 * (1.0f / _sampleFreq);
            c += qDot3 * (1.0f / _sampleFreq);
            d += qDot4 * (1.0f / _sampleFreq);

            // Normalise quaternion
            recipNorm = invSqrt(a * a + b * b + c * c + d * d);
            a *= recipNorm;
            b *= recipNorm;
            c *= recipNorm;
            d *= recipNorm;

        }

        float invSqrt(float x) {
            float halfx = 0.5f * x;
            float y = x;
            long i = *(long*)&y;
            i = 0x5f3759df - (i>>1);
            y = *(float*)&i;
            y = y * (1.5f - (halfx * y * y));
            return y;
        }

        void printRot() {
            float ux, uy, uz, vx, vy, vz, wx, wy, wz, uu, vv, ww, uv, uw, vw;
            ux = a*a + b*b - c*c - d*d;
            uy = 2*b*c + 2*a*d;
            uz = 2*b*d - 2*a*c;

            vx = 2*b*c - 2*a*d;
            vy = a*a - b*b + c*c - d*d;
            vz = 2*c*d + 2*a*b;

            wx = 2*b*d + 2*a*c;
            wy = 2*c*d - 2*a*b;
            wz = a*a - b*b - c*c + d*d;

            // co-polarization of basis vectors
            uu = ux*ux + uy*uy + uz*uz;
            vv = vx*vx + vy*vy + vz*vz;
            ww = wx*wx + wy*wy + wz*wz;

            // cross-polarization of basis vectors
            uv = ux*vx + uy*vy + uz*vz;
            uw = ux*wx + uy*wy + uz*wz;
            vw = vx*wx + vy*wy + vz*wz;

            printf("u[%+.3f,%+.3f,%+.3f] v[%+.3f,%+.3f,%+.3f] w[%+.3f,%+.3f,%+.3f] copol[%+.3f,%+.3f,%+.3f] xpol[%+.3f,%+.3f,%+.3f] ",ux,uy,uz,vx,vy,vz,wx,wy,wz,uu,vv,ww,uv,uw,vw);
        }


        void print() {
            printf("q0: %+.3f, q1: %+.3f, q2: %+.3f, q3: %+.3f, norm: %+.3f, ", a, b, c, d, sqrt(a*a + b*b + c*c + d*d));
        }
};
