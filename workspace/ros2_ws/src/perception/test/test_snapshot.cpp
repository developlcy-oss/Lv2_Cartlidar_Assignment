// 팔 카메라 사진 저장 일정 · 파일 이름 · 쓰기 스레드 (snapshot.hpp)
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <thread>

#include "perception/snapshot.hpp"

using perception::JobQueue;
using perception::SnapshotConfig;
using perception::SnapshotSchedule;
using A = SnapshotSchedule::Action;

// 기본 3 장 · 0.5 s 뒤부터 0.5 s 간격: 영상이 계속 오면 0.5 · 1.0 · 1.5 s 에 한 장씩
TEST(SnapshotSchedule, ThreeShotsOnTime)
{
  SnapshotSchedule s;
  EXPECT_FALSE(s.start("/x/images/01_before_r1_B1", 10.0, 5));
  EXPECT_EQ(s.poll(10.4, 9), A::Wait);
  EXPECT_EQ(s.poll(10.5, 9), A::Take);
  EXPECT_EQ(s.taken(9), 1);
  EXPECT_EQ(s.poll(10.9, 12), A::Wait);
  EXPECT_EQ(s.poll(11.0, 12), A::Take);
  EXPECT_EQ(s.taken(12), 2);
  EXPECT_EQ(s.poll(11.5, 15), A::Take);
  EXPECT_EQ(s.taken(15), 3);
  EXPECT_FALSE(s.active());
  EXPECT_EQ(s.poll(12.0, 18), A::Idle);
}

// 요청 뒤 새 영상이 없으면 찍지 않고 기다림 (구독을 막 켠 경우), 오면 그때 찍음
TEST(SnapshotSchedule, WaitsForNewFrame)
{
  SnapshotSchedule s;
  s.start("p", 0.0, 7);
  EXPECT_EQ(s.poll(0.6, 7), A::Wait);
  EXPECT_EQ(s.poll(2.0, 8), A::Take);
  EXPECT_EQ(s.taken(8), 1);
  // 같은 영상을 두 번 쓰지 않음
  EXPECT_EQ(s.poll(2.1, 8), A::Wait);
}

// 그 장 시각 + max_wait_s 동안 새 영상이 없으면 남은 장 포기
TEST(SnapshotSchedule, GivesUpAfterMaxWait)
{
  SnapshotSchedule s;
  s.start("p", 0.0, 1);
  EXPECT_EQ(s.poll(0.5, 2), A::Take);
  s.taken(2);
  EXPECT_EQ(s.poll(3.9, 2), A::Wait);    // 2 번째 장 시각 1.0 + 3.0 = 4.0 까지
  EXPECT_EQ(s.poll(4.01, 2), A::GiveUp);
  EXPECT_EQ(s.givenUpAt(), 2);
  EXPECT_FALSE(s.active());
}

// 끝나기 전에 새 요청이 오면 true (남은 장 버림) 를 돌려주고 새 요청부터
TEST(SnapshotSchedule, NewRequestCutsOld)
{
  SnapshotSchedule s;
  s.start("a", 0.0, 1);
  s.poll(0.5, 2);
  s.taken(2);
  EXPECT_TRUE(s.start("b", 0.6, 2));
  EXPECT_EQ(s.prefix(), "b");
  EXPECT_EQ(s.next(), 1);
  EXPECT_EQ(s.poll(1.0, 3), A::Wait);
  EXPECT_EQ(s.poll(1.1, 3), A::Take);
}

TEST(SnapshotSchedule, CountZeroDisabled)
{
  SnapshotConfig c;
  c.count = 0;
  SnapshotSchedule s(c);
  s.start("p", 0.0, 0);
  EXPECT_EQ(s.poll(5.0, 9), A::Idle);
}

// 이미 있는 파일은 덮어쓰지 않고 _2, _3 을 붙임
TEST(SnapshotPath, DoesNotOverwrite)
{
  namespace fs = std::filesystem;
  const fs::path dir = fs::temp_directory_path() / ("snapshot_test_" + std::to_string(::getpid()));
  fs::remove_all(dir);
  fs::create_directories(dir);
  const std::string prefix = (dir / "07_before_r1_B3").string();
  EXPECT_EQ(perception::unique_jpg_path(prefix, 1), prefix + "_1.jpg");
  std::ofstream(prefix + "_1.jpg") << "x";
  EXPECT_EQ(perception::unique_jpg_path(prefix, 1), prefix + "_1_2.jpg");
  std::ofstream(prefix + "_1_2.jpg") << "x";
  EXPECT_EQ(perception::unique_jpg_path(prefix, 1), prefix + "_1_3.jpg");
  EXPECT_EQ(perception::unique_jpg_path(prefix, 2), prefix + "_2.jpg");
  fs::remove_all(dir);
}

// 작업은 넣은 순서대로, 소멸 때 남은 작업까지 다 함
TEST(JobQueue, RunsInOrderAndDrains)
{
  std::vector<int> done;
  std::mutex m;
  {
    JobQueue q;
    for (int i = 0; i < 5; ++i) {
      q.push([&, i]() {
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
          std::lock_guard<std::mutex> lk(m);
          done.push_back(i);
        });
    }
  }
  ASSERT_EQ(done.size(), 5u);
  for (int i = 0; i < 5; ++i) {EXPECT_EQ(done[i], i);}
}
