/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2019 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
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
 *
 */

/**
 * scara.cpp
 */

#include "../inc/MarlinConfig.h"

#if IS_SCARA

#include "scara.h"
#include "motion.h"
#include "planner.h"

float delta_segments_per_second = SCARA_SEGMENTS_PER_SECOND;

void scara_set_axis_is_at_home(const AxisEnum axis) {
    /**
     * SCARA homes XYZ at the same time
     */
    xyz_pos_t homeposition;
    LOOP_XYZ(i) homeposition[i] = base_home_pos((AxisEnum)i);

    #if ENABLED(MORGAN_SCARA)
      // MORGAN_SCARA uses arm angles for AB home position
      // SERIAL_ECHOLNPAIR("homeposition A:", homeposition.a, " B:", homeposition.b);
      inverse_kinematics(homeposition);
      forward_kinematics_SCARA(delta.a, delta.b);
      current_position[axis] = cartes[axis];
    #else
      // MP_SCARA uses a Cartesian XY home position
      // SERIAL_ECHOLNPAIR("homeposition X:", homeposition.x, " Y:", homeposition.y);
      forward_kinematics_SCARA(homeposition[X_AXIS], homeposition[Y_AXIS],homeposition[Z_AXIS]);
      current_position[axis] = cartes[axis];
    #endif
    
    // SERIAL_ECHOLNPAIR("Cartesian X:", current_position.x, " Y:", current_position.y);
    update_software_endstops(axis);
  
}

static constexpr xyz_pos_t scara_offset = { SCARA_OFFSET_X, SCARA_OFFSET_Y, SCARA_OFFSET_Z};

/**
 * Morgan SCARA Forward Kinematics. Results in 'cartes'.
 * Maths and first version by QHARLEY.
 * Integrated into Marlin and slightly restructured by Joachim Cerny.
 */
void forward_kinematics_SCARA(const float &a, const float &b, const float &c) {

  const float a_sin = sin(RADIANS(a)) * L1,
              a_cos = cos(RADIANS(a)) * L1,
              b_sin = sin(RADIANS(b)) * L2,
              b_cos = cos(RADIANS(b)) * L2;
  float arm_xy = a_cos + b_cos + CENTER_OFFSET + HEAD_OFFSET;

  cartes.set(arm_xy * cos(c/SCARA_RAD2DEG) - scara_offset[X_AXIS],
             arm_xy * sin(c/SCARA_RAD2DEG) - scara_offset[Y_AXIS],
             a_sin + b_sin - scara_offset[Z_AXIS]); 
	// SERIAL_ECHOLNPAIR(" angle (X,Y,Z) = (",a, ", ", b,", ", c, ")");
  // SERIAL_ECHOLNPAIR(" cartes (X,Y,Z) = (",cartes.x, ", ", cartes.y,", ", cartes.z, ")");
  /*
    SERIAL_ECHOLNPAIR(
      "SCARA FK Angle a=", a,
      " b=", b,
      " a_sin=", a_sin,
      " a_cos=", a_cos,
      " b_sin=", b_sin,
      " b_cos=", b_cos
    );
    SERIAL_ECHOLNPAIR(" cartes (X,Y) = "(cartes.x, ", ", cartes.y, ")");
  //*/
}

void inverse_kinematics(const xyz_pos_t &raw) {

  #if ENABLED(MORGAN_SCARA)
    /**
     * Morgan SCARA Inverse Kinematics. Results in 'delta'.
     *
     * See http://forums.reprap.org/read.php?185,283327
     *
     * Maths and first version by QHARLEY.
     * Integrated into Marlin and slightly restructured by Joachim Cerny.
     */
    float C2, S2, SK1, SK2, THETA, PSI;

    // Translate SCARA to standard XY with scaling factor
    const xy_pos_t spos = raw - scara_offset;

    const float H2 = HYPOT2(spos.x, spos.y);
    if (L1 == L2)
      C2 = H2 / L1_2_2 - 1;
    else
      C2 = (H2 - (L1_2 + L2_2)) / (2.0f * L1 * L2);

    S2 = SQRT(1.0f - sq(C2));

    // Unrotated Arm1 plus rotated Arm2 gives the distance from Center to End
    SK1 = L1 + L2 * C2;

    // Rotated Arm2 gives the distance from Arm1 to Arm2
    SK2 = L2 * S2;

    // Angle of Arm1 is the difference between Center-to-End angle and the Center-to-Elbow
    THETA = ATAN2(SK1, SK2) - ATAN2(spos.x, spos.y);

    // Angle of Arm2
    PSI = ATAN2(S2, C2);

    delta.set(DEGREES(THETA), DEGREES(THETA + PSI), raw.z);

    /*
      DEBUG_POS("SCARA IK", raw);
      DEBUG_POS("SCARA IK", delta);
      SERIAL_ECHOLNPAIR("  SCARA (x,y) ", sx, ",", sy, " C2=", C2, " S2=", S2, " Theta=", THETA, " Phi=", PHI);
    //*/

  #else // MP_SCARA
    float SCARA_offset[3]={SCARA_OFFSET_X,SCARA_OFFSET_Y,SCARA_OFFSET_Z};
    float SCARA_pos[3];
    static float SCARA_C2, SCARA_S2, SCARA_K1, SCARA_K2, SCARA_theta, SCARA_psi; 
    static float ARM_XYZ, ARM_XY;
    float TempDelta;
    float Linkage_1 = SCARA_LINKAGE_1;
    float Linkage_2 = SCARA_LINKAGE_2;
    
    SCARA_pos[X_AXIS] = raw.x + SCARA_offset[X_AXIS];  //Translate SCARA to standard X Y
    SCARA_pos[Y_AXIS] = raw.y + SCARA_offset[Y_AXIS];  // With scaling factor.
    SCARA_pos[Z_AXIS] = raw.z + SCARA_offset[Z_AXIS];

    
    ARM_XY = sqrt(pow(SCARA_pos[X_AXIS],2) + pow(SCARA_pos[Y_AXIS],2)) - CENTER_OFFSET - HEAD_OFFSET;
    ARM_XYZ = sqrt(pow(ARM_XY,2) + pow(SCARA_pos[Z_AXIS],2));
    
    SCARA_C2 = (pow(ARM_XYZ,2) - pow(Linkage_1,2) - pow(Linkage_2,2))/(2 * Linkage_1 * Linkage_2);
    
    SCARA_S2 = sqrt( 1 - pow(SCARA_C2,2) );
    
    SCARA_K1 = Linkage_1 + Linkage_2 * SCARA_C2;
    SCARA_K2 = Linkage_2 * SCARA_S2;
    
    SCARA_theta = (atan2(SCARA_pos[Z_AXIS],ARM_XY)+atan2(SCARA_K2, SCARA_K1));
    SCARA_psi   =  atan2(SCARA_S2, SCARA_C2);
    
    float thrtaA = SCARA_theta * SCARA_RAD2DEG;  // Multiply by 180/Pi  -  theta is support arm angle
    float thrtaB = (SCARA_theta - SCARA_psi) * SCARA_RAD2DEG;  //       -  equal to sub arm angle (inverted motor)
    float thrtaC = atan2(SCARA_pos[Y_AXIS],SCARA_pos[X_AXIS]) * SCARA_RAD2DEG;
  

    delta.set(thrtaA, thrtaB, thrtaC);

    /*
      DEBUG_POS("SCARA IK", raw);
      DEBUG_POS("SCARA IK", delta);
      SERIAL_ECHOLNPAIR("  SCARA (x,y) ", x, ",", y," Theta1=", THETA1, " Theta2=", THETA2);
    //*/
    //  SERIAL_ECHOLNPAIR("  SCARA (x,y,z) ", raw.x, ",", raw.y,",",raw.z, " ThetaA=", thrtaA, " ThetaB=", thrtaB, " ThetaC=", thrtaC);

  #endif // MP_SCARA
}

void scara_report_positions() {
  SERIAL_ECHOLNPAIR("  SCARA (x,y,z) ", current_position[X_AXIS], ",", current_position[Y_AXIS],",",current_position[Z_AXIS],"\nSCARA ThetaA:", planner.get_axis_position_degrees(A_AXIS), "  ThetaB:", planner.get_axis_position_degrees(B_AXIS), "  ThetaC:", planner.get_axis_position_degrees(C_AXIS));
  SERIAL_EOL();
}

#endif // IS_SCARA
