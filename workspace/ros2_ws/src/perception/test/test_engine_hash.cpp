// 엔진 무결성 검사 시험 (engine_hash.hpp). 해시 값은 FIPS 180-4 예제와 sha256sum 결과
#include <gtest/gtest.h>

#include <string>

#include "perception/engine_hash.hpp"

using namespace perception;

TEST(EngineHash, KnownVectors)
{
  EXPECT_EQ(sha256_hex("", 0), "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
  EXPECT_EQ(sha256_hex("abc", 3), "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
  const std::string two = "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq";
  EXPECT_EQ(sha256_hex(two.data(), two.size()), "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1");
  const std::string million(1000000, 'a');
  EXPECT_EQ(sha256_hex(million.data(), million.size()), "cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0");
}

TEST(EngineHash, BlockBoundaries)
{
  // 패딩이 블록 하나 · 둘로 갈리는 길이
  EXPECT_NE(sha256_hex(std::string(55, 'x').data(), 55), sha256_hex(std::string(56, 'x').data(), 56));
  EXPECT_NE(sha256_hex(std::string(63, 'x').data(), 63), sha256_hex(std::string(64, 'x').data(), 64));
}

TEST(EngineHash, FlippedByteChangesHash)
{
  std::string a(4096, 'q');
  const std::string before = sha256_hex(a.data(), a.size());
  a[2000] ^= 0x01;
  EXPECT_NE(sha256_hex(a.data(), a.size()), before);
}

TEST(EngineHash, ParseFile)
{
  const std::string h = "d7cea24d68d94bd94ba27995f14deb3105d90dbac9b97f0a3fb3df1f2b58345f";
  EXPECT_EQ(parse_sha256_file(h + "  yolo.engine\n"), h);
  EXPECT_EQ(parse_sha256_file("\n " + h), h);
  std::string up = h;
  for (auto & c : up) {c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));}
  EXPECT_EQ(parse_sha256_file(up + " x"), h);
  EXPECT_EQ(parse_sha256_file(""), "");
  EXPECT_EQ(parse_sha256_file("abc  yolo.engine"), "");
  EXPECT_EQ(parse_sha256_file(h.substr(0, 63) + "z  yolo.engine"), "");
}
