// 미션 기록 폴더 (에피소드 = /mission/start 로 새로 시작한 미션 하나). ROS 의존 없음 (시험용으로 분리)
//   <log_dir>/<YYMMDD_HHMM>/ 하나에 mission_log.jsonl · images/ · planner/ · 퍽 등록표 사본을 모은다.
//   같은 분에 다시 시작해 폴더가 이미 있으면 _2, _3 ... 을 붙인다 (덮어쓰지 않음)
#pragma once

#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <string>

namespace mission
{

// 앞 '~' 또는 '~/' 만 HOME 으로 (yaml · ros2 param 은 '~' 를 풀지 않음). HOME 이 없으면 그대로
inline std::string expandHome(const std::string & path)
{
  if (path.empty() || path[0] != '~' || (path.size() > 1 && path[1] != '/')) {return path;}
  const char * home = std::getenv("HOME");
  if (!home || !*home) {return path;}
  return std::string(home) + path.substr(1);
}

// 벽시계 (지역 시각) YYMMDD_HHMM. sim_time 이어도 벽시계를 쓴다 (sim 시계로는 1970 이 됨)
inline std::string episodeName(std::time_t t)
{
  std::tm tm{};
  localtime_r(&t, &tm);
  char buf[32];
  std::strftime(buf, sizeof(buf), "%y%m%d_%H%M", &tm);
  return buf;
}

// <root>/<name>, 이미 있으면 <root>/<name>_2 ... 를 만들어 돌려줌. 못 만들면 빈 문자열 (err 에 이유)
inline std::string makeEpisodeDir(const std::string & root, const std::string & name, std::string & err)
{
  namespace fs = std::filesystem;
  std::error_code ec;
  fs::create_directories(root, ec);
  if (ec) {
    err = "cannot create " + root + ": " + ec.message();
    return "";
  }
  for (int n = 1; n < 1000; ++n) {
    const fs::path p = fs::path(root) / (n == 1 ? name : name + "_" + std::to_string(n));
    if (fs::create_directory(p, ec)) {return p.string();}   // 새로 만든 경우만 (있던 폴더는 건너뜀)
    if (ec) {
      err = "cannot create " + p.string() + ": " + ec.message();
      return "";
    }
  }
  err = "too many episode folders named " + name + " in " + root;
  return "";
}

// 사진 접두어 <episode>/images/<NN>_<what> (NN = 에피소드 안 요청 번호, 두 자리). perception 이 _1.jpg ... 를 붙인다
inline std::string snapshotPrefix(const std::string & episode, int seq, const std::string & what)
{
  char nn[16];
  std::snprintf(nn, sizeof(nn), "%02d", seq);
  return (std::filesystem::path(episode) / "images" / (std::string(nn) + "_" + what)).string();
}

// src 폴더의 파일 (하위 폴더 제외) 을 dst 로 복사 (덮어씀). 복사한 수, 못 하면 -1 (err 에 이유)
inline int copyFiles(const std::string & src, const std::string & dst, std::string & err)
{
  namespace fs = std::filesystem;
  std::error_code ec;
  if (!fs::is_directory(src, ec)) {
    err = "no folder " + src;
    return -1;
  }
  fs::create_directories(dst, ec);
  if (ec) {
    err = "cannot create " + dst + ": " + ec.message();
    return -1;
  }
  int n = 0;
  for (const auto & e : fs::directory_iterator(src, ec)) {
    if (!e.is_regular_file()) {continue;}
    fs::copy_file(e.path(), fs::path(dst) / e.path().filename(), fs::copy_options::overwrite_existing, ec);
    if (ec) {
      err = "cannot copy " + e.path().string() + ": " + ec.message();
      return -1;
    }
    ++n;
  }
  if (ec) {
    err = "cannot list " + src + ": " + ec.message();
    return -1;
  }
  return n;
}

// 파일 하나를 dst 폴더로 복사 (이름 유지, 덮어씀)
inline bool copyFileInto(const std::string & file, const std::string & dst, std::string & err)
{
  namespace fs = std::filesystem;
  std::error_code ec;
  fs::create_directories(dst, ec);
  fs::copy_file(file, fs::path(dst) / fs::path(file).filename(), fs::copy_options::overwrite_existing, ec);
  if (ec) {
    err = "cannot copy " + file + " -> " + dst + ": " + ec.message();
    return false;
  }
  return true;
}

}  // namespace mission
