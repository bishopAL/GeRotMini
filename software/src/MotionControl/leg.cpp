#include "leg.h"
/****************legStatus**********************/
Leg::Leg(string name,float L1,float L2,float L3);
{
    _name=name;
    _L1=L1;
    _L2=L2;
    _L3=L3;

}
 void Leg::setJointPos(vector<float> jointPos)
 {
    _theta0=jointPos[0];
    _theta1=jointPos[1];
    _theta2=jointPos[2];
 }

 void changeStatus(__legStatus legStatus)
 {
    _legStatus=legStatus;
 }

__legStatus Leg::getLegStatus()
{
    return _legStatus;
}
/**************Kinematics*******************/
/**
 * @brief 
 * Calculcate jacobian_vector with jointPresPos
 */
void Leg::updateJacobians()
{
    
       Matrix<float, 3, 3>  CHANGE; CHANGE << 0, 0, 1,
        1, 0, 0,
        0, 1, 0;
        float factor_y, factor_z, factor_By, factor_Cy, factor_Bz, factor_Cz, factor_Ax, factor_Bx;
        switch (_name)
        {
        case "LF":
            factor_By = -1;
            factor_Cy = 1;
            factor_Bz = 1;
            factor_Cz = 1;
            factor_Ax = -1;
            factor_Bx = -1;
            factor_y = 1;
            factor_z = 1;
            break;
        case "RF":
            factor_By = 1;
            factor_Cy = -1;
            factor_Bz = -1;
            factor_Cz = -1;
            factor_Ax = 1;
            factor_Bx = -1;
            factor_y = -1;
            factor_z = -1;
            break;
        case "LH":
            factor_By = 1;
            factor_Cy = 1;
            factor_Bz = -1;
            factor_Cz = 1;
            factor_Ax = -1;
            factor_Bx = 1;
            factor_y = 1;
            factor_z = 1;
            break;
        case "RH":
            factor_By = -1;
            factor_Cy = -1;
            factor_Bz = 1;
            factor_Cz = -1;
            factor_Ax = 1;
            factor_Bx = 1;
            factor_y = -1;
            factor_z = -1;
            break;
        default:
            break;
        }
        _jacobian(0, 0) = factor_y * (-sin(_theta1) * cos(_theta2) * L1 + factor_By * sin(_theta1) * sin(_theta2 + _theta3) * L2 + factor_Cy* cos(_theta1) * L3);
        _jacobian(0, 1) = factor_y * (-cos(_theta1) * sin(_theta2) * L1 - factor_By * cos(_theta1) * cos(_theta2 + _theta3) * L2);
        _jacobian(0, 2) = factor_y * (-factor_By * cos(_theta1) * cos(_theta2 + _theta3) * L2);
        _jacobian(1, 0) = factor_z * (cos(_theta1) * cos(_theta2) * L1 + factor_Bz * cos(_theta1) * sin(_theta2 + _theta3) * L2 + factor_Cz * sin(_theta1) * L3);
        _jacobian(1, 1) = factor_z * (-sin(_theta1) * sin(_theta2) * L1 + factor_Bz * sin(_theta1) * cos(_theta2 + _theta3) * L2);
        _jacobian(1, 2) = factor_z * (factor_Bz * sin(_theta1) * cos(_theta2 + _theta3) * L2);
        _jacobian(2, 0) = 0;
        _jacobian(2, 1) = factor_Ax * cos(_theta2) * L1 + factor_Bx * sin(_theta2 + _theta3) * L2;
        _jacobian(2, 2) = factor_Bx * sin(_theta2 + _theta3) * L2;
        _jacobian = CHANGE * _jacobian;
    
}

Matrix<float,3,1> MotionControl::forwardKinematics(int mode)
{
        Matrix<float,3,1> legPos;
        float factor_Ay, factor_By, factor_Cy, factor_Az, factor_Bz, factor_Cz, factor_Ax, factor_Bx;//The sign factors before the formulas L1, L2, and L3, where A represents the sign factors before L1, B represents the sign factors before L2, C represents the sign factors before L3
        float factor_y, factor_z;//the sign factors of whole formulas of y,z
        switch (_name)
        {
        case "LF":
            factor_Ay = 1;
            factor_By = 1;
            factor_Cy = 1;
            factor_Az = 1;
            factor_Bz = 1;
            factor_Cz = -1;
            factor_Ax = -1;
            factor_Bx = 1;
            factor_y = 1;
            factor_z = 1;
            break;
        case "RF":
            factor_Ay = 1;
            factor_By = -1;
            factor_Cy = -1;
            factor_Az = 1;
            factor_Bz = -1;
            factor_Cz = 1;
            factor_Ax = 1;
            factor_Bx = 1;
            factor_y = -1;
            factor_z = -1;
            break;
        case "LH":
            factor_Ay = 1;
            factor_By = -1;
            factor_Cy = 1;
            factor_Az = 1;
            factor_Bz = -1;
            factor_Cz = -1;
            factor_Ax = -1;
            factor_Bx = -1;
            factor_y = 1;
            factor_z = 1;
            break;
        case "RH":
            factor_Ay = 1;
            factor_By = 1;
            factor_Cy = -1;
            factor_Az = 1;
            factor_Bz = 1;
            factor_Cz = 1;
            factor_Ax = 1;
            factor_Bx = -1;
            factor_y = -1;
            factor_z = -1;
            break;
        default:
            break;
        }
        // represent y,z,x
        legPos(1,0) = factor_y * (factor_Ay * cos(_theta1) * cos(_theta2) * L1 + factor_By * cos(_theta1) * sin(_theta2 + _theta3) * L2 + factor_Cy*sin(_theta1) * L3);
        legPos(2,0) = factor_z * (factor_Az * sin(_theta1) * cos(_theta2) * L1 + factor_Bz * sin(_theta1) * sin(_theta2 + _theta3) * L2 + factor_Cz * cos(_theta1) * L3);
        legPos(0,0) = factor_Ax * sin(_theta2) * L1 + factor_Bx * cos(_theta2 + _theta3) * L2;
        return legPos;
}

/**
 * @brief inverse Kinematics
 * 
 * @param cmdpos 
 * Calculcate joint angles (jointCmdPos) for motors with foot position(cmdpos) in shoulder coordinate
 */
Matrix<float,3,1> Leg::inverseKinematics(Matrix<float, 4, 3> cmdpos)
{
        Matrix<float,3,1> jointCmdPos;
        float factor_x, factor_y, factor_z, factor_1, factor_2, factor_0;
        float x, y, z;
        x = cmdpos(legNum, 0);
        y = cmdpos(legNum, 1);
        z = cmdpos(legNum, 2);
        switch (_name)
        {
        case "LF":
            factor_x = -1;
            factor_y = 1;
            factor_z = 1;
            factor_1 = 1;
            factor_2 = -1;
            factor_0 = 1;
            break;
        case "RF":
            factor_x = 1;
            factor_y = -1;
            factor_z = -1;
            factor_1 = -1;
            factor_2 = 1;
            factor_0 = -1;
            break;
        case "LH":
            factor_x = -1;
            factor_y = 1;
            factor_z = 1;
            factor_1 = 1;
            factor_2 = 1;
            factor_0 = -1;
            break;
        case "RH":
            factor_x = 1;
            factor_y = -1;
            factor_z = -1;
            factor_1 = -1;
            factor_2 = -1;
            factor_0 = 1;
            break;

        }
        // jointCmdPos(legNum, 1) = atan2(factor_z * z, factor_y * y) + factor_1 * atan2(L3, sqrt(z * z + y * y - L3 * L3));
        // jointCmdPos(legNum, 2) = factor_2 * asin((L1 * L1 + L2 * L2 + L3 * L3 - x * x - y * y - z * z) / (2 * L1 * L2));
        // jointCmdPos(legNum, 0) = atan2(factor_x * x, sqrt((L1 + factor_0 * sin(jointCmdPos(legNum, 2)) * L2) * (L1 + factor_0 * sin(jointCmdPos(legNum, 2)) * L2) + (cos(jointCmdPos(legNum, 2)) * L2) * (cos(jointCmdPos(legNum, 2)) * L2) - x * x)) + factor_0 * atan2(cos(jointCmdPos(legNum, 2)) * L2, L1 + factor_0 * L2 * sin(jointCmdPos(legNum, 2)));
         jointCmdPos(0, 0) = atan2(factor_z * z, factor_y * y) + factor_1 * atan2(L3, sqrt(z * z + y * y - L3 * L3));
         jointCmdPos(1, 0) = atan2(factor_x * x, sqrt((L1 + factor_0 * sin(jointCmdPos(legNum, 2)) * L2) * (L1 + factor_0 * sin(jointCmdPos(legNum, 2)) * L2) + (cos(jointCmdPos(legNum, 2)) * L2) * (cos(jointCmdPos(legNum, 2)) * L2) - x * x)) + factor_0 * atan2(cos(jointCmdPos(legNum, 2)) * L2, L1 + factor_0 * L2 * sin(jointCmdPos(legNum, 2)));
         jointCmdPos(2, 0) = factor_2 * asin((L1 * L1 + L2 * L2 + L3 * L3 - x * x - y * y - z * z) / (2 * L1 * L2));
         return jointCmdPos;
    
}


