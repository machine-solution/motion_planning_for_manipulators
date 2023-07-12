#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include <cstdio>

TEST_CASE("JointState comparation")
{
    JointState a({1, 2});
    JointState b({3, -4});
    JointState c({5});
    CHECK(a < b);
    CHECK(c < a);
    CHECK(c < b);
    CHECK(a > c);
    CHECK(a <= b);
    CHECK(c != a);
    CHECK(b >= c);
    CHECK(b >= JointState({2, 2}));
    CHECK(c == JointState({5}));
}




TEST_CASE("Integration test")
{
    CHECK();
}