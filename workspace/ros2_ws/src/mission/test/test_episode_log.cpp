// 미션 기록 폴더 · 사진 접두어 · 사본 (episode_log.hpp)
#include <gtest/gtest.h>

#include <unistd.h>

#include <filesystem>
#include <fstream>

#include "mission/episode_log.hpp"

namespace fs = std::filesystem;
using namespace mission;

class TmpDir : public ::testing::Test
{
protected:
  void SetUp() override
  {
    dir = fs::temp_directory_path() / ("episode_log_test_" + std::to_string(::getpid()));
    fs::remove_all(dir);
  }
  void TearDown() override {fs::remove_all(dir);}
  fs::path dir;
};

TEST(ExpandHome, OnlyLeadingTilde)
{
  ::setenv("HOME", "/home/u", 1);
  EXPECT_EQ(expandHome("~/Lv2/mission_log"), "/home/u/Lv2/mission_log");
  EXPECT_EQ(expandHome("~"), "/home/u");
  EXPECT_EQ(expandHome("/tmp/mission_log"), "/tmp/mission_log");
  EXPECT_EQ(expandHome("~other/x"), "~other/x");   // 다른 사용자 홈은 풀지 않음
  EXPECT_EQ(expandHome(""), "");
}

TEST(EpisodeName, LocalWallClock)
{
  std::tm tm{};
  tm.tm_year = 2026 - 1900;
  tm.tm_mon = 10;   // 11 월
  tm.tm_mday = 10;
  tm.tm_hour = 9;
  tm.tm_min = 5;
  tm.tm_isdst = -1;
  EXPECT_EQ(episodeName(std::mktime(&tm)), "261110_0905");
}

// 같은 이름이 있으면 _2, _3 (이전 미션 폴더를 덮어쓰지 않음), 없는 상위 폴더도 만듦
TEST_F(TmpDir, EpisodeDirDoesNotReuse)
{
  std::string err;
  const std::string root = (dir / "a" / "mission_log").string();
  EXPECT_EQ(makeEpisodeDir(root, "261110_0905", err), root + "/261110_0905");
  EXPECT_EQ(makeEpisodeDir(root, "261110_0905", err), root + "/261110_0905_2");
  EXPECT_EQ(makeEpisodeDir(root, "261110_0905", err), root + "/261110_0905_3");
  EXPECT_EQ(makeEpisodeDir(root, "261110_0906", err), root + "/261110_0906");
}

TEST_F(TmpDir, EpisodeDirFailsOnFile)
{
  fs::create_directories(dir);
  std::ofstream(dir / "file") << "x";
  std::string err;
  EXPECT_EQ(makeEpisodeDir((dir / "file").string(), "n", err), "");
  EXPECT_FALSE(err.empty());
}

TEST(SnapshotPrefix, NumberedInImages)
{
  EXPECT_EQ(snapshotPrefix("/r/261110_0905", 7, "before_r1_B3"), "/r/261110_0905/images/07_before_r1_B3");
  EXPECT_EQ(snapshotPrefix("/r/e", 12, "after_L2"), "/r/e/images/12_after_L2");
}

// 계획기 기록 폴더의 파일만 복사 (하위 폴더 제외), 없는 폴더는 -1
TEST_F(TmpDir, CopyFiles)
{
  const fs::path src = dir / "planner", dst = dir / "ep" / "planner";
  std::string err;
  EXPECT_EQ(copyFiles(src.string(), dst.string(), err), -1);
  fs::create_directories(src / "sub");
  for (const char * f : {"signal.json", "allocation.json", "plan.json", "mission_lines.txt"}) {std::ofstream(src / f) << f;}
  std::ofstream(src / "sub" / "x") << "x";
  EXPECT_EQ(copyFiles(src.string(), dst.string(), err), 4);
  EXPECT_TRUE(fs::exists(dst / "plan.json"));
  EXPECT_FALSE(fs::exists(dst / "sub"));
  // 다시 복사하면 덮어씀
  std::ofstream(src / "plan.json") << "new";
  EXPECT_EQ(copyFiles(src.string(), dst.string(), err), 4);
  std::ifstream in(dst / "plan.json");
  std::string s;
  in >> s;
  EXPECT_EQ(s, "new");
}

TEST_F(TmpDir, CopyFileInto)
{
  fs::create_directories(dir);
  std::ofstream(dir / "pucks_survey.yaml") << "pucks: []";
  std::string err;
  EXPECT_TRUE(copyFileInto((dir / "pucks_survey.yaml").string(), (dir / "ep").string(), err));
  EXPECT_TRUE(fs::exists(dir / "ep" / "pucks_survey.yaml"));
  EXPECT_FALSE(copyFileInto((dir / "none.yaml").string(), (dir / "ep").string(), err));
}
