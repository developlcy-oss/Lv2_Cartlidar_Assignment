// 측량 완료 파일 -> 퍽 등록표 (survey_pucks.hpp): pose 옮기기 (#193), 없으면 안 씀, 틀린 값 · 칸 수 오류
#include <gtest/gtest.h>

#include <string>

#include "mission/survey_pucks.hpp"

using mission::surveyToPucksYaml;

namespace
{
// 12 칸 (박스 1 ~ 6, 양쪽). k 번째 칸만 pose_k 로, 나머지는 standing. pose_k 가 "-" 면 그 칸에 pose 키가 없음
YAML::Node survey(int k = -1, const std::string & pose_k = "standing")
{
  YAML::Node doc;
  for (int i = 0; i < 12; ++i) {
    YAML::Node s;
    s["box"] = i / 2 + 1;
    s["side"] = i % 2 ? 1 : -1;
    s["id"] = i;
    s["color"] = i % 2 ? "red" : "green";
    s["shape"] = i < 6 ? "cylinder" : "cuboid";
    if (i != k) {
      s["pose"] = "standing";
    } else if (pose_k == "null") {
      s["pose"] = YAML::Null;
    } else if (pose_k != "-") {
      s["pose"] = pose_k;
    }
    doc["slots"].push_back(s);
  }
  return doc;
}

int count(const std::string & s, const std::string & sub)
{
  int n = 0;
  for (auto p = s.find(sub); p != std::string::npos; p = s.find(sub, p + 1)) {++n;}
  return n;
}
}  // namespace

TEST(SurveyPucks, CopiesPose)
{
  std::string out, err;
  ASSERT_TRUE(surveyToPucksYaml(survey(7, "lying"), "f.json", out, err)) << err;
  EXPECT_NE(out.find("  - {id: 7, color: red, shape: box, box: 4, side: 1, pose: lying}\n"), std::string::npos) << out;
  EXPECT_NE(out.find("  - {id: 0, color: green, shape: cylinder, box: 1, side: -1, pose: standing}\n"), std::string::npos);
  EXPECT_EQ(count(out, "pose: lying"), 1);
  EXPECT_EQ(count(out, "pose: standing"), 11);
  EXPECT_EQ(count(out, "shape: cuboid"), 0);  // 계획기 이름 box
  // 다시 읽으면 12 개
  const YAML::Node back = YAML::Load(out);
  ASSERT_EQ(back["pucks"].size(), 12u);
  EXPECT_EQ(back["pucks"][7]["pose"].as<std::string>(), "lying");
}

TEST(SurveyPucks, MissingOrNullPoseOmitted)
{
  for (const std::string p : {"-", "null"}) {
    std::string out, err;
    ASSERT_TRUE(surveyToPucksYaml(survey(3, p), "f.json", out, err)) << p << " " << err;
    EXPECT_NE(out.find("  - {id: 3, color: red, shape: cylinder, box: 2, side: 1}\n"), std::string::npos) << out;
    EXPECT_EQ(count(out, "pose:"), 11);
  }
}

TEST(SurveyPucks, RejectsBadPoseAndSlotCount)
{
  std::string out, err;
  EXPECT_FALSE(surveyToPucksYaml(survey(2, "sideways"), "f.json", out, err));
  EXPECT_NE(err.find("standing or lying"), std::string::npos) << err;

  YAML::Node doc = survey();
  doc["slots"].remove(0);
  err.clear();
  EXPECT_FALSE(surveyToPucksYaml(doc, "f.json", out, err));
  EXPECT_NE(err.find("12 slots"), std::string::npos) << err;

  YAML::Node noid = survey();
  noid["slots"][5]["id"] = YAML::Null;
  err.clear();
  EXPECT_FALSE(surveyToPucksYaml(noid, "f.json", out, err));
  EXPECT_NE(err.find("id · color · shape"), std::string::npos) << err;
}
