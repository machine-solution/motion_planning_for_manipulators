#include "light_mujoco.h"

/*
This code is copied out from src mujoco and refactoring to be
faster in collision checks which we need. Use this code only
for collision checkings.
*/

// assign/override contact margin
mjtNum mj_assignMargin(const mjModel* m, mjtNum source) {
  if (mjENABLED(mjENBL_OVERRIDE)) {
    return m->opt.o_margin;
  } else {
    return source;
  }
}

// plane to geom_center squared distance, g1 is a plane
static mjtNum plane_geom(const mjModel* m, mjData* d, int g1, int g2) {
  mjtNum* mat1 = d->geom_xmat + 9*g1;
  mjtNum norm[3] = {mat1[2], mat1[5], mat1[8]};
  mjtNum dif[3];

  mju_sub3(dif, d->geom_xpos + 3*g2, d->geom_xpos + 3*g1);
  return mju_dot3(dif, norm);
}

// squared Euclidean distance between 3D vectors
static inline mjtNum squaredDist3(const mjtNum pos1[3], const mjtNum pos2[3]) {
  mjtNum dif[3] = {pos1[0]-pos2[0], pos1[1]-pos2[1], pos1[2]-pos2[2]};
  return dif[0]*dif[0] + dif[1]*dif[1] + dif[2]*dif[2];
}

// bounding-sphere collision
static int mj_collideSphere(const mjModel* m, mjData* d, int g1, int g2, mjtNum margin) {
  // neither geom is a plane
  if (m->geom_rbound[g1]>0 && m->geom_rbound[g2]>0) {
    mjtNum bound = m->geom_rbound[g1] + m->geom_rbound[g2] + margin;
    if (squaredDist3(d->geom_xpos+3*g1, d->geom_xpos+3*g2) > bound*bound) {
      return 0;
    }
  }

  // one geom is a plane
  if (m->geom_type[g1]==mjGEOM_PLANE && m->geom_rbound[g2]>0
      && plane_geom(m, d, g1, g2) > margin+m->geom_rbound[g2]) {
      return 0;
  }
  if (m->geom_type[g2]==mjGEOM_PLANE && m->geom_rbound[g1]>0
      && plane_geom(m, d, g2, g1) > margin+m->geom_rbound[g1]) {
      return 0;
  }
  return 1;
}

bool mj_light_collideGeoms(const mjModel* m, mjData* d, int g1, int g2) {
    int num, type1, type2, condim;
    mjtNum margin, gap, mix, friction[5], solref[mjNREF], solimp[mjNIMP];
    mjContact con[mjMAXCONPAIR];
    int ipair = (g2<0 ? g1 : -1);

    // get explicit geom ids from pair
    if (ipair>=0) {
        g1 = m->pair_geom1[ipair];
        g2 = m->pair_geom2[ipair];
    }

    // order geoms by type
    if (m->geom_type[g1] > m->geom_type[g2]) {
        std::swap(g1, g2);
    }

    // copy types and bodies
    type1 = m->geom_type[g1];
    type2 = m->geom_type[g2];

    // return if no collision function
    if (!mjCOLLISIONFUNC[type1][type2]) {
        return false;
    }
    margin = m->pair_margin[ipair];
    gap = m->pair_gap[ipair];
    condim = m->pair_dim[ipair];

    // bounding sphere filter
    if (!mj_collideSphere(m, d, g1, g2, margin)) {
        return false;
    }

    // call collision detector to generate contacts
    num = mjCOLLISIONFUNC[type1][type2](m, d, con, g1, g2, margin);
    return num;
}

// use only for predefined format
bool mj_light_collision(mjModel* m, mjData* d)
{
    mj_kinematics(m, d);

    for (int pairadr = 0; pairadr < m->npair; pairadr++)
    {
        if (mj_light_collideGeoms(m, d, pairadr, -1))
        {
            return true;
        }
    }
    return false;
}
