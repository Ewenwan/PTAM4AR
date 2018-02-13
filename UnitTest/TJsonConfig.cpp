#include "JsonConfig.h"

#include <gtest/gtest.h>

TEST(JsonConfig,basis)
{
    JsonConfig jsonConfig("../../Config.json");
    ASSERT_EQ(GS::RET_SUCESS, jsonConfig.Init());

    int nMaxSSD = jsonConfig.GetInt("Tracker.MiniPatchMaxSSD");
    std::cout << "nMaxSSD: " << nMaxSSD << std::endl;
}
