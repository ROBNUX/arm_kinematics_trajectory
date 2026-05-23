// Smoke tests for the kinematic map classes. These verify that each concrete
// BaseKinematicMap subclass constructs, reports the expected name, and that
// polymorphic dispatch through a BaseKinematicMap pointer works.

#include <gtest/gtest.h>

#include <memory>

#include "robnux_kinematics_map/UJNT.hpp"
#include "robnux_kinematics_map/XYZ.hpp"
#include "robnux_kinematics_map/XYZ_UR.hpp"
#include "robnux_kinematics_map/base_kinematics.hpp"
#include "robnux_kinematics_map/quattro.hpp"
#include "robnux_kinematics_map/quattro_4.hpp"
#include "robnux_kinematics_map/quattroK.hpp"
#include "robnux_kinematics_map/scara.hpp"
#include "robnux_kinematics_map/singleAxisModule.hpp"
#include "robnux_kinematics_map/sixaxis_1.hpp"

namespace {

using kinematics_lib::BaseKinematicMap;
using kinematics_lib::Quattro;
using kinematics_lib::Quattro_4;
using kinematics_lib::QuattroK;
using kinematics_lib::Scara;
using kinematics_lib::singleAxisModule;
using kinematics_lib::SixAxis_1;
using kinematics_lib::UJNT;
using kinematics_lib::XYZ_UR;
using kinematics_lib::XYZGantry;

TEST(KinematicMaps, ScaraConstructsAndReportsName) {
  Scara s;
  EXPECT_EQ(s.GetName(), "Scara");
}

TEST(KinematicMaps, SixAxis1ConstructsAndReportsName) {
  SixAxis_1 s;
  EXPECT_EQ(s.GetName(), "SixAxis_1");
}

TEST(KinematicMaps, UjntConstructsAndReportsName) {
  UJNT u;
  EXPECT_EQ(u.GetName(), "UJNT");
}

TEST(KinematicMaps, XyzGantryConstructsAndReportsName) {
  XYZGantry x;
  EXPECT_EQ(x.GetName(), "XYZGantry");
}

TEST(KinematicMaps, SingleAxisModuleConstructsAndReportsName) {
  singleAxisModule m;
  EXPECT_EQ(m.GetName(), "singleAxisModule");
}

TEST(KinematicMaps, XyzUrConstructsAndReportsName) {
  XYZ_UR x;
  EXPECT_EQ(x.GetName(), "XYZ_UR");
}

TEST(KinematicMaps, QuattroConstructsAndReportsName) {
  Quattro q;
  EXPECT_EQ(q.GetName(), "Quattro");
}

TEST(KinematicMaps, Quattro4ConstructsAndReportsName) {
  Quattro_4 q;
  EXPECT_EQ(q.GetName(), "Quattro_4");
}

TEST(KinematicMaps, QuattroKConstructsAndReportsName) {
  QuattroK q;
  EXPECT_EQ(q.GetName(), "QuattroK");
}

TEST(KinematicMaps, BaseKinematicMapPolymorphism) {
  // Upcast through BaseKinematicMap* and dispatch the virtual GetName.
  std::unique_ptr<BaseKinematicMap> bot(new Scara());
  EXPECT_EQ(bot->GetName(), "Scara");
}

}  // namespace
