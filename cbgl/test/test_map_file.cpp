// Copyright 2026 Alexandros Filotheou
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

/**
 * @brief `cbgl::mapFromFile`: the map a description file on disk describes, or
 * a sentence saying why it describes none.
 * @details
 * **Every case writes its own files and takes them away again.** A description
 * and a picture are two files that have to agree, and half the failures worth
 * catching here are disagreements between them, so a case that cannot write a
 * broken pair cannot ask the question it exists to ask. Nothing is committed as
 * a fixture for the same reason: a checked in PGM is a file nobody can read
 * without a tool, and what it is meant to hold would then live only in whatever
 * comment stands beside it.
 * **The picture is asymmetric on purpose.** A PGM's first row is the top of the
 * picture and an occupancy grid's first row is the one at the origin, which is
 * the bottom, so the rows have to be turned over on the way in. A picture that
 * read the same either way would pass whether they were or not, and the map that
 * mistake produces is a legal map of a building that does not exist: every
 * answer found in it is confidently wrong, and nothing downstream can notice.
 * **The expected cells are written down rather than computed.** The conversion
 * from a sample to an occupancy is the whole of what is under test, so a case
 * that recomputed it would agree with the code by construction, correct or not.
 * The six values are hand checked against the map server's own trinary rule:
 * a sample of 0 is a wall, 255 is floor, and the halfway grey is a cell nobody
 * has seen.
 * **Both variants of PGM are decoded.** Binary `P5` is what ROS map saving
 * emits, plain `P2` is the same picture in text, and the two are held to the
 * same grid rather than each to its own expectation.
 * **A picture is read on its own terms.** A PGM states what its own white is,
 * and 255 is only the commonest choice. The same room is written here at a
 * maximum of 15 and at a maximum of 65535 as well, and the same map is required
 * out of all three, because a decoder that reads every sample as a fraction of
 * 255 whatever the file declared turns a picture declaring 15 into a building
 * with no floor left in it. That failure is the worst kind: no error, no
 * warning, a map that is wrong everywhere, and a localiser that then cannot
 * find a pose in it for reasons nobody can see.
 * @remark A refusal has to name the file and, where it is a key that is
 * missing or malformed, the key. That is asserted of every refusal here. A map
 * that failed to load is read by an operator hours later out of a log, and
 * "cannot read map" with neither a path nor a key in it costs that operator the
 * afternoon.
 */

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <filesystem>
#include <format>
#include <fstream>
#include <ios>
#include <string>
#include <vector>

#include <geometry_msgs/msg/quaternion.hpp>

#include "cbgl/map_file.hpp"

namespace
{
using cbgl::mapFromFile;

const std::int8_t kBlocked = 100;
const std::int8_t kFree = 0;
const std::int8_t kUnknown = -1;

const double kOriginX = -1.35;
const double kOriginY = 2.70;
const double kOriginYaw = 0.60;
const double kResolution = 0.05;

const unsigned int kColumns = 3U;
const unsigned int kRows = 2U;

const std::string kDescriptionName = "room.yaml";
const std::string kPictureName = "room.pgm";

/**
 * @brief The samples of the picture every case that needs a readable one
 * writes, in the order a PGM carries them: the top row first.
 */
std::vector<unsigned char> everySample()
{
  return {0U, 255U, 128U,
          255U, 128U, 0U};
}

/**
 * @brief The cells the picture above describes, in the order an occupancy grid
 * carries them: the row at the origin first, which is the bottom of the picture.
 */
std::vector<std::int8_t> everyCell()
{
  return {kFree, kUnknown, kBlocked,
          kBlocked, kFree, kUnknown};
}

/**
 * @brief The cells the same picture describes once it is negated, which is the
 * same reading with the wall and the floor exchanged.
 */
std::vector<std::int8_t> everyNegatedCell()
{
  return {kBlocked, kUnknown, kFree,
          kFree, kBlocked, kUnknown};
}

/**
 * @brief The heading a quaternion of a map's origin carries, about z.
 */
double yawOf(const geometry_msgs::msg::Quaternion& orientation)
{
  return 2.0 * std::atan2(orientation.z, orientation.w);
}

class MapFile : public testing::Test
{
protected:
  void SetUp() override
  {
    directory_ = std::filesystem::temp_directory_path() / std::format(
      "cbgl_map_file_{}",
      testing::UnitTest::GetInstance()->current_test_info()->name());

    std::filesystem::remove_all(directory_);
    std::filesystem::create_directories(directory_);
  }

  void TearDown() override
  {
    std::filesystem::remove_all(directory_);
  }

  /**
   * @brief The bytes of a binary PGM of the samples given.
   */
  std::string binaryPgm(const unsigned int columns, const unsigned int rows,
    const unsigned int largest, const std::vector<unsigned char>& samples) const
  {
    std::string content = std::format("P5\n{} {}\n{}\n", columns, rows,
      largest);

    for (const unsigned char sample : samples)
      content.push_back(static_cast<char>(sample));

    return content;
  }

  /**
   * @brief The three keys a description must carry, naming the picture every
   * case writes beside it.
   */
  std::string everyRequiredKey() const
  {
    return std::format("image: {}\nresolution: {}\norigin: [{}, {}, {}]\n",
      kPictureName, kResolution, kOriginX, kOriginY, kOriginYaw);
  }

  /**
   * @brief The text of a plain PGM of the samples given.
   */
  std::string plainPgm(const unsigned int columns, const unsigned int rows,
    const unsigned int largest, const std::vector<unsigned char>& samples) const
  {
    std::string content = std::format("P2\n{} {}\n{}\n", columns, rows,
      largest);

    for (const unsigned char sample : samples)
      content += std::format("{}\n", static_cast<unsigned int>(sample));

    return content;
  }

  /**
   * @brief The bytes of a binary PGM of two bytes a sample, the more
   * significant first, which is the order the format specifies.
   */
  std::string wideBinaryPgm(const unsigned int columns, const unsigned int rows,
    const unsigned int largest, const std::vector<unsigned int>& samples) const
  {
    std::string content = std::format("P5\n{} {}\n{}\n", columns, rows,
      largest);

    for (const unsigned int sample : samples)
    {
      content.push_back(static_cast<char>((sample >> 8) & 0xFFU));
      content.push_back(static_cast<char>(sample & 0xFFU));
    }

    return content;
  }

  /**
   * @brief Write one file of this case's own directory.
   * @return where it was written.
   */
  std::filesystem::path write(const std::string& name,
    const std::string& content) const
  {
    const std::filesystem::path path = directory_ / name;
    std::ofstream file(path, std::ios::binary);
    file << content;
    file.close();

    return path;
  }

  /**
   * @brief Write the picture every case that needs a readable one shares, as a
   * binary PGM beside the description.
   */
  void writePicture() const
  {
    write(kPictureName, binaryPgm(kColumns, kRows, 255U, everySample()));
  }

  std::filesystem::path directory_;
};

}

/**
 * @brief The comment a saved PGM carries is not part of its header.
 * @details A map saved by ROS writes a line naming what saved it between the
 * magic number and the width, so a decoder that reads the header as three
 * numbers separated by whitespace refuses the very files this exists to read.
 */
TEST_F(MapFile, ACommentInThePicturesHeaderIsSkipped)
{
  const std::vector<unsigned char> samples = everySample();

  write(kPictureName, std::format("P5\n# saved by something\n{} {}\n255\n",
    kColumns, kRows) + std::string(samples.begin(), samples.end()));

  const std::filesystem::path description =
    write(kDescriptionName, everyRequiredKey());

  const std::expected<nav_msgs::msg::OccupancyGrid, std::string> map =
    mapFromFile(description.string());

  ASSERT_TRUE(map.has_value()) << map.error();
  EXPECT_EQ(map->data, everyCell());
}

/**
 * @brief A plain picture that stops before its size is refused, as a binary
 * one already is.
 * @details The two variants take separate paths through the decoder and only
 * the binary one had a case, so a plain picture could have been truncated
 * silently for as long as nobody wrote this.
 */
TEST_F(MapFile, APlainPictureCarryingTooFewSamplesIsRefused)
{
  write(kPictureName, std::format("P2\n{} {}\n255\n0 0 0\n", kColumns,
    kRows));

  const std::filesystem::path description =
    write(kDescriptionName, everyRequiredKey());

  const std::expected<nav_msgs::msg::OccupancyGrid, std::string> map =
    mapFromFile(description.string());

  ASSERT_FALSE(map.has_value());
  EXPECT_NE(map.error().find(kPictureName), std::string::npos) << map.error();
}

/**
 * @brief A size no file could carry is refused before anything is sized from
 * it.
 * @details A picture states its own dimensions and nothing in it obliges them
 * to describe what follows. Sizing a buffer from them first turns a mistyped
 * digit into an allocation of everything the machine has, so the header is
 * measured against the bytes in hand before a byte is reserved.
 */
TEST_F(MapFile, ASizeTheFileCannotCarryIsRefusedRatherThanAllocated)
{
  write(kPictureName, std::string("P5\n4294967295 4294967295\n255\nX"));

  const std::filesystem::path description =
    write(kDescriptionName, everyRequiredKey());

  const std::expected<nav_msgs::msg::OccupancyGrid, std::string> map =
    mapFromFile(description.string());

  ASSERT_FALSE(map.has_value());
  EXPECT_NE(map.error().find(kPictureName), std::string::npos) << map.error();
}

/**
 * @brief A magic number that is neither variant is refused by name.
 */
TEST_F(MapFile, APictureOfAnotherFormatIsRefusedByNamingTheFile)
{
  write(kPictureName, std::string("P6\n3 2\n255\nxxxxxxxxxxxxxxxxxx"));

  const std::filesystem::path description =
    write(kDescriptionName, everyRequiredKey());

  const std::expected<nav_msgs::msg::OccupancyGrid, std::string> map =
    mapFromFile(description.string());

  ASSERT_FALSE(map.has_value());
  EXPECT_NE(map.error().find(kPictureName), std::string::npos) << map.error();
}

/**
 * @brief A resolution or an origin that is not a finite number is refused by
 * naming the key.
 * @details Neither survives contact with the arithmetic that places a pose: a
 * resolution of nought divides by nought in the cell indexing and a NaN in the
 * origin poisons every answer the node will ever give, both far from the file
 * that caused it.
 */
TEST_F(MapFile, ANumberThatIsNotFiniteIsRefusedByNamingItsKey)
{
  writePicture();

  const std::filesystem::path bad_resolution = write("nan_resolution.yaml",
    std::format("image: {}\nresolution: .nan\norigin: [{}, {}, {}]\n",
      kPictureName, kOriginX, kOriginY, kOriginYaw));

  const std::expected<nav_msgs::msg::OccupancyGrid, std::string> by_resolution =
    mapFromFile(bad_resolution.string());

  ASSERT_FALSE(by_resolution.has_value());
  EXPECT_NE(by_resolution.error().find("resolution"), std::string::npos)
    << by_resolution.error();

  const std::filesystem::path bad_origin = write("nan_origin.yaml",
    std::format("image: {}\nresolution: {}\norigin: [{}, .nan, {}]\n",
      kPictureName, kResolution, kOriginX, kOriginYaw));

  const std::expected<nav_msgs::msg::OccupancyGrid, std::string> by_origin =
    mapFromFile(bad_origin.string());

  ASSERT_FALSE(by_origin.has_value());
  EXPECT_NE(by_origin.error().find("origin"), std::string::npos)
    << by_origin.error();
}

/**
 * @brief A threshold outside nought to one, or the two of them the wrong way
 * round, is refused by naming the key.
 * @details These two decide what every cell of the map means. A threshold
 * above one leaves nothing blocked and a free threshold above the occupied one
 * leaves no cell unknown, and both are silent.
 */
TEST_F(MapFile, AThresholdOutsideItsRangeIsRefusedByNamingItsKey)
{
  writePicture();

  const std::filesystem::path too_large = write("large_threshold.yaml",
    everyRequiredKey() + "occupied_thresh: 2.0\n");

  const std::expected<nav_msgs::msg::OccupancyGrid, std::string> by_range =
    mapFromFile(too_large.string());

  ASSERT_FALSE(by_range.has_value());
  EXPECT_NE(by_range.error().find("occupied_thresh"), std::string::npos)
    << by_range.error();

  const std::filesystem::path inverted = write("inverted_thresholds.yaml",
    everyRequiredKey() + "occupied_thresh: 0.2\nfree_thresh: 0.8\n");

  const std::expected<nav_msgs::msg::OccupancyGrid, std::string> by_order =
    mapFromFile(inverted.string());

  ASSERT_FALSE(by_order.has_value());
  EXPECT_NE(by_order.error().find("free_thresh"), std::string::npos)
    << by_order.error();
}

/**
 * @brief A description asking for a mode this does not read is refused rather
 * than given the one it did not ask for.
 * @details The map server reads three modes and they give three different maps
 * from one picture. This reads trinary.
 */
TEST_F(MapFile, AModeThisDoesNotReadIsRefusedByNamingIt)
{
  writePicture();

  const std::filesystem::path description = write(kDescriptionName,
    everyRequiredKey() + "mode: raw\n");

  const std::expected<nav_msgs::msg::OccupancyGrid, std::string> map =
    mapFromFile(description.string());

  ASSERT_FALSE(map.has_value());
  EXPECT_NE(map.error().find("raw"), std::string::npos) << map.error();
}

/**
 * @brief An image that is not an ordinary file is refused rather than read.
 * @details A path naming a character device reads until the memory runs out
 * and one naming a pipe blocks start up for ever, which is a node that never
 * comes up and never says why.
 */
TEST_F(MapFile, AnImageThatIsNotAnOrdinaryFileIsRefused)
{
  const std::filesystem::path description = write(kDescriptionName,
    std::format("image: /dev/zero\nresolution: {}\norigin: [{}, {}, {}]\n",
      kResolution, kOriginX, kOriginY, kOriginYaw));

  const std::expected<nav_msgs::msg::OccupancyGrid, std::string> map =
    mapFromFile(description.string());

  ASSERT_FALSE(map.has_value());
  EXPECT_NE(map.error().find("/dev/zero"), std::string::npos) << map.error();
}

TEST_F(MapFile, ADescriptionOfNoImageIsRefusedByNamingTheKeyAndTheFile)
{
  writePicture();

  const std::filesystem::path description = write(kDescriptionName,
    std::format("resolution: {}\norigin: [0.0, 0.0, 0.0]\n", kResolution));

  const std::expected<nav_msgs::msg::OccupancyGrid, std::string> map =
    mapFromFile(description.string());

  ASSERT_FALSE(map.has_value())
    << "a description naming no image described a map anyway, so a file that "
    << "says nothing about which picture it means was read as though it did";

  EXPECT_NE(map.error().find("image"), std::string::npos) << map.error();
  EXPECT_NE(map.error().find(description.string()), std::string::npos)
    << map.error();
}

TEST_F(MapFile, ADescriptionOfNoOriginIsRefusedByNamingTheKeyAndTheFile)
{
  writePicture();

  const std::filesystem::path description = write(kDescriptionName,
    std::format("image: {}\nresolution: {}\n", kPictureName, kResolution));

  const std::expected<nav_msgs::msg::OccupancyGrid, std::string> map =
    mapFromFile(description.string());

  ASSERT_FALSE(map.has_value())
    << "a description placing the map nowhere was read as placing it "
    << "somewhere, and every answer from it would be wrong by wherever the "
    << "map really is";

  EXPECT_NE(map.error().find("origin"), std::string::npos) << map.error();
  EXPECT_NE(map.error().find(description.string()), std::string::npos)
    << map.error();
}

TEST_F(MapFile, ADescriptionOfNoResolutionIsRefusedByNamingTheKeyAndTheFile)
{
  writePicture();

  const std::filesystem::path description = write(kDescriptionName,
    std::format("image: {}\norigin: [0.0, 0.0, 0.0]\n", kPictureName));

  const std::expected<nav_msgs::msg::OccupancyGrid, std::string> map =
    mapFromFile(description.string());

  ASSERT_FALSE(map.has_value())
    << "a description stating no resolution was read as stating one, and the "
    << "map it describes would be the right shape at the wrong scale";

  EXPECT_NE(map.error().find("resolution"), std::string::npos) << map.error();
  EXPECT_NE(map.error().find(description.string()), std::string::npos)
    << map.error();
}

TEST_F(MapFile, ADescriptionThatIsNotThereIsRefusedByNamingTheFile)
{
  const std::filesystem::path description = directory_ / kDescriptionName;

  const std::expected<nav_msgs::msg::OccupancyGrid, std::string> map =
    mapFromFile(description.string());

  ASSERT_FALSE(map.has_value());

  EXPECT_NE(map.error().find(description.string()), std::string::npos)
    << map.error();
}

/**
 * @brief A description that is not a mapping at all.
 * @details A path pointed at the wrong file is the commonest way this happens,
 * and the wrong file is far likelier to be a list or a sentence than a mapping
 * of keys nobody asked for.
 */
TEST_F(MapFile, ADescriptionThatStatesNoKeysAtAllIsRefusedByNamingTheFile)
{
  const std::filesystem::path description =
    write(kDescriptionName, "- room\n- corridor\n");

  const std::expected<nav_msgs::msg::OccupancyGrid, std::string> map =
    mapFromFile(description.string());

  ASSERT_FALSE(map.has_value());

  EXPECT_NE(map.error().find(description.string()), std::string::npos)
    << map.error();
}

/**
 * @brief A maximum no PGM may state.
 * @details The format allows two bytes a sample and no more, so 65535 is the
 * ceiling. A file stating more than that is stating something no reader agrees
 * on, and guessing at the sample width would be reading whichever half of each
 * pixel the guess happened to land on.
 */
TEST_F(MapFile, AMaximumAboveWhatThePgmFormatAllowsIsRefused)
{
  write(kPictureName, binaryPgm(kColumns, kRows, 65536U, everySample()));

  const std::filesystem::path description =
    write(kDescriptionName, everyRequiredKey());

  const std::expected<nav_msgs::msg::OccupancyGrid, std::string> map =
    mapFromFile(description.string());

  ASSERT_FALSE(map.has_value());

  EXPECT_NE(map.error().find("65536"), std::string::npos) << map.error();
  EXPECT_NE(map.error().find("65535"), std::string::npos) << map.error();
}

/**
 * @brief The same room, written at a maximum of 15 rather than of 255.
 * @details Every sample is a fraction of the maximum its own picture states, so
 * a 15 in a picture declaring 15 is the white a 255 is in a picture declaring
 * 255. What is asserted is that the two describe the very same map, which is
 * the property a decoder dividing by a hardcoded 255 breaks: it would read this
 * picture's floor as almost black and hand back a room with no free space in it
 * at all, without a word said.
 */
TEST_F(MapFile, AMaximumBelowAByteIsRescaledOntoTheWholeRange)
{
  write(kPictureName, binaryPgm(kColumns, kRows, 15U,
    {0U, 15U, 8U, 15U, 8U, 0U}));

  const std::filesystem::path description =
    write(kDescriptionName, everyRequiredKey());

  const std::expected<nav_msgs::msg::OccupancyGrid, std::string> narrow =
    mapFromFile(description.string());

  ASSERT_TRUE(narrow.has_value()) << narrow.error();

  EXPECT_EQ(narrow->data, everyCell())
    << "a picture stating a maximum of 15 was read as though its samples were "
    << "fractions of 255, so its white came out nearly black and its floor "
    << "came out a wall";

  write(kPictureName, binaryPgm(kColumns, kRows, 255U,
    {0U, 255U, 136U, 255U, 136U, 0U}));

  const std::expected<nav_msgs::msg::OccupancyGrid, std::string> wide =
    mapFromFile(description.string());

  ASSERT_TRUE(wide.has_value()) << wide.error();

  EXPECT_EQ(narrow->data, wide->data)
    << "the same room written at two maxima came back as two different maps, "
    << "so which map an operator gets depends on what a saving tool happened "
    << "to declare";
}

TEST_F(MapFile, AMaximumOfNoughtIsRefused)
{
  write(kPictureName, binaryPgm(kColumns, kRows, 0U, everySample()));

  const std::filesystem::path description =
    write(kDescriptionName, everyRequiredKey());

  const std::expected<nav_msgs::msg::OccupancyGrid, std::string> map =
    mapFromFile(description.string());

  ASSERT_FALSE(map.has_value())
    << "a picture whose white and whose black are the same value was read as a "
    << "map, and every sample of it would be divided by nothing";

  EXPECT_NE(map.error().find("maximum sample value of 0"), std::string::npos)
    << map.error();
}

TEST_F(MapFile, APictureCarryingFewerSamplesThanItsSizeNeedsIsRefused)
{
  const std::vector<unsigned char> samples = everySample();
  const std::vector<unsigned char> half(samples.begin(), samples.begin() + 2);

  write(kPictureName, binaryPgm(kColumns, kRows, 255U, half));

  const std::filesystem::path description =
    write(kDescriptionName, everyRequiredKey());

  const std::expected<nav_msgs::msg::OccupancyGrid, std::string> map =
    mapFromFile(description.string());

  ASSERT_FALSE(map.has_value())
    << "a truncated picture was read as a whole one, so the map carries "
    << "fewer cells than its own width and height require and nothing "
    << "downstream can index it";

  EXPECT_NE(map.error().find(kPictureName), std::string::npos) << map.error();
}

TEST_F(MapFile, AResolutionOfNoLengthAtAllIsRefusedByNamingTheKey)
{
  writePicture();

  const std::filesystem::path description = write(kDescriptionName,
    std::format("image: {}\nresolution: 0.0\norigin: [0.0, 0.0, 0.0]\n",
      kPictureName));

  const std::expected<nav_msgs::msg::OccupancyGrid, std::string> map =
    mapFromFile(description.string());

  ASSERT_FALSE(map.has_value())
    << "a resolution of zero was accepted, and a map of cells no wide is a "
    << "map every position in the world falls in the same cell of";

  EXPECT_NE(map.error().find("resolution"), std::string::npos) << map.error();
  EXPECT_NE(map.error().find(description.string()), std::string::npos)
    << map.error();
}

TEST_F(MapFile, AResolutionThatIsNotANumberIsRefusedByNamingTheKey)
{
  writePicture();

  const std::filesystem::path description = write(kDescriptionName,
    std::format("image: {}\nresolution: five centimetres\n"
      "origin: [0.0, 0.0, 0.0]\n", kPictureName));

  const std::expected<nav_msgs::msg::OccupancyGrid, std::string> map =
    mapFromFile(description.string());

  ASSERT_FALSE(map.has_value())
    << "a resolution that is not a number was read as one";

  EXPECT_NE(map.error().find("resolution"), std::string::npos) << map.error();
  EXPECT_NE(map.error().find(description.string()), std::string::npos)
    << map.error();
}

/**
 * @brief A picture that contradicts its own header.
 * @details A sample above the maximum the same picture states cannot be
 * rescaled onto a byte without leaving it, so it is refused in both variants
 * rather than wrapped round into whatever the arithmetic produces.
 */
TEST_F(MapFile, ASampleAboveThePicturesOwnMaximumIsRefusedInEitherVariant)
{
  const std::filesystem::path description =
    write(kDescriptionName, everyRequiredKey());

  write(kPictureName, plainPgm(kColumns, kRows, 15U,
    {0U, 15U, 20U, 15U, 8U, 0U}));

  const std::expected<nav_msgs::msg::OccupancyGrid, std::string> plain =
    mapFromFile(description.string());

  ASSERT_FALSE(plain.has_value());
  EXPECT_NE(plain.error().find("20"), std::string::npos) << plain.error();

  write(kPictureName, binaryPgm(kColumns, kRows, 15U,
    {0U, 15U, 20U, 15U, 8U, 0U}));

  const std::expected<nav_msgs::msg::OccupancyGrid, std::string> binary =
    mapFromFile(description.string());

  ASSERT_FALSE(binary.has_value());
  EXPECT_NE(binary.error().find("20"), std::string::npos) << binary.error();
}

/**
 * @brief Two bytes a sample, and which of the two comes first.
 * @details A maximum above 255 means two bytes a sample and the format puts the
 * more significant one first. The sample 0xFF00 is what decides the case: read
 * the right way round it is very nearly white and the cell is free, and read
 * the wrong way round it is 255 out of 65535, which is nearly black and the
 * cell is a wall. So a decoder that swapped the two bytes fails here rather
 * than passing on a picture that happened to be symmetric.
 */
TEST_F(MapFile, ATwoBytePictureIsReadWithTheMoreSignificantByteFirst)
{
  write(kPictureName, wideBinaryPgm(kColumns, kRows, 65535U,
    {0x0000U, 0xFF00U, 0x8888U, 0xFF00U, 0x8888U, 0x0000U}));

  const std::filesystem::path description =
    write(kDescriptionName, everyRequiredKey());

  const std::expected<nav_msgs::msg::OccupancyGrid, std::string> map =
    mapFromFile(description.string());

  ASSERT_TRUE(map.has_value()) << map.error();

  EXPECT_EQ(map->info.width, kColumns);
  EXPECT_EQ(map->info.height, kRows);

  EXPECT_EQ(map->data, everyCell())
    << "a two byte picture came back as a different map from the one byte "
    << "picture of the same room, so either the samples were paired the wrong "
    << "way round or they were not rescaled by the maximum stated";
}

/**
 * @brief The whole of what a valid pair describes.
 * @details The origin is translated and turned, because a map recorded of a
 * real building has both and a map read as though it had neither is a map every
 * answer is wrong in.
 */
TEST_F(MapFile, AValidDescriptionAndPictureBecomeTheMapTheyDescribe)
{
  writePicture();

  const std::filesystem::path description =
    write(kDescriptionName, everyRequiredKey());

  const std::expected<nav_msgs::msg::OccupancyGrid, std::string> map =
    mapFromFile(description.string());

  ASSERT_TRUE(map.has_value()) << map.error();

  EXPECT_EQ(map->info.width, kColumns);
  EXPECT_EQ(map->info.height, kRows);
  EXPECT_NEAR(map->info.resolution, kResolution, 1e-9);

  EXPECT_NEAR(map->info.origin.position.x, kOriginX, 1e-9);
  EXPECT_NEAR(map->info.origin.position.y, kOriginY, 1e-9);
  EXPECT_NEAR(map->info.origin.position.z, 0.0, 1e-9);
  EXPECT_NEAR(yawOf(map->info.origin.orientation), kOriginYaw, 1e-9)
    << "the origin's heading is what turns the map's own cells into world "
    << "positions, so a map read flat is a map rotated by however much the "
    << "building really is";

  EXPECT_EQ(map->data, everyCell());

  EXPECT_TRUE(map->header.frame_id.empty())
    << "the map came back labelled \"" << map->header.frame_id << "\", and a "
    << "description file states no frame. Whoever asked for the map is who "
    << "knows which frame it is expressed in";
}

TEST_F(MapFile, AnAbsoluteImagePathIsTakenAsItStands)
{
  const std::filesystem::path elsewhere = directory_ / "pictures";
  std::filesystem::create_directories(elsewhere);

  const std::filesystem::path picture = elsewhere / kPictureName;
  std::ofstream file(picture, std::ios::binary);
  file << binaryPgm(kColumns, kRows, 255U, everySample());
  file.close();

  const std::filesystem::path description = write(kDescriptionName,
    std::format("image: {}\nresolution: {}\norigin: [{}, {}, {}]\n",
      picture.string(), kResolution, kOriginX, kOriginY, kOriginYaw));

  const std::expected<nav_msgs::msg::OccupancyGrid, std::string> map =
    mapFromFile(description.string());

  ASSERT_TRUE(map.has_value()) << map.error();
  EXPECT_EQ(map->data, everyCell());
}

TEST_F(MapFile, AnImageThatIsNotAPgmIsRefusedByNamingBothVariantsThatAre)
{
  write(kPictureName, std::string("\x89PNG\r\n\x1a\n") + "and then nothing");

  const std::filesystem::path description =
    write(kDescriptionName, everyRequiredKey());

  const std::expected<nav_msgs::msg::OccupancyGrid, std::string> map =
    mapFromFile(description.string());

  ASSERT_FALSE(map.has_value())
    << "a file that is not a PGM was decoded as one, so whatever its bytes "
    << "happen to be became a map";

  EXPECT_NE(map.error().find("P5"), std::string::npos) << map.error();
  EXPECT_NE(map.error().find("P2"), std::string::npos) << map.error();
  EXPECT_NE(map.error().find(kPictureName), std::string::npos) << map.error();
}

TEST_F(MapFile, AnImageThatIsNotThereIsRefusedByNamingIt)
{
  const std::filesystem::path description =
    write(kDescriptionName, everyRequiredKey());

  const std::expected<nav_msgs::msg::OccupancyGrid, std::string> map =
    mapFromFile(description.string());

  ASSERT_FALSE(map.has_value());

  EXPECT_NE(map.error().find(kPictureName), std::string::npos) << map.error();
}

TEST_F(MapFile, AnOriginOfOtherThanThreeValuesIsRefusedByNamingTheKey)
{
  writePicture();

  const std::filesystem::path description = write(kDescriptionName,
    std::format("image: {}\nresolution: {}\norigin: [1.0, 2.0]\n",
      kPictureName, kResolution));

  const std::expected<nav_msgs::msg::OccupancyGrid, std::string> map =
    mapFromFile(description.string());

  ASSERT_FALSE(map.has_value())
    << "an origin of two values was read as three, so the map is placed at "
    << "whatever the third turned out to be";

  EXPECT_NE(map.error().find("origin"), std::string::npos) << map.error();
  EXPECT_NE(map.error().find(description.string()), std::string::npos)
    << map.error();
}

/**
 * @brief A description carries what it carries and this reads what it reads.
 * @details A map saved by ROS states its mode and its negate flag whether
 * anything reads them or not, and a description written by hand collects
 * whatever its author found useful. None of that is grounds for refusing a map.
 */
TEST_F(MapFile, AnythingElseTheDescriptionCarriesIsIgnored)
{
  writePicture();

  const std::filesystem::path description = write(kDescriptionName,
    everyRequiredKey() + "mode: trinary\nsurveyed_by: nobody\n");

  const std::expected<nav_msgs::msg::OccupancyGrid, std::string> map =
    mapFromFile(description.string());

  ASSERT_TRUE(map.has_value()) << map.error();
  EXPECT_EQ(map->data, everyCell());
}

/**
 * @brief The rows are turned over, and the picture says which way.
 * @details Held here on its own, beside the case above that asserts the whole
 * map, so a failure says which of the two properties broke. The first row of
 * the grid is the bottom row of the picture and the last is the top.
 */
TEST_F(MapFile, TheFirstRowOfTheMapIsTheBottomRowOfThePicture)
{
  writePicture();

  const std::filesystem::path description =
    write(kDescriptionName, everyRequiredKey());

  const std::expected<nav_msgs::msg::OccupancyGrid, std::string> map =
    mapFromFile(description.string());

  ASSERT_TRUE(map.has_value()) << map.error();
  ASSERT_EQ(map->data.size(), static_cast<std::size_t>(kColumns) * kRows);

  const std::vector<std::int8_t> cells = everyCell();

  for (unsigned int column = 0U; column < kColumns; column++)
  {
    EXPECT_EQ(map->data[column], cells[column])
      << "the map's first row is not the picture's last, so the map is a "
      << "reflection of the building it was recorded of, which is a legal map "
      << "nothing downstream can tell from the real one";
  }
}

TEST_F(MapFile, TheNegateFlagExchangesWhatIsBlockedForWhatIsFree)
{
  writePicture();

  const std::filesystem::path description =
    write(kDescriptionName, everyRequiredKey() + "negate: 1\n");

  const std::expected<nav_msgs::msg::OccupancyGrid, std::string> map =
    mapFromFile(description.string());

  ASSERT_TRUE(map.has_value()) << map.error();
  EXPECT_EQ(map->data, everyNegatedCell());
}

TEST_F(MapFile, ThePlainVariantOfPgmDescribesTheSameMapAsTheBinaryOne)
{
  write(kPictureName, plainPgm(kColumns, kRows, 255U, everySample()));

  const std::filesystem::path description =
    write(kDescriptionName, everyRequiredKey());

  const std::expected<nav_msgs::msg::OccupancyGrid, std::string> map =
    mapFromFile(description.string());

  ASSERT_TRUE(map.has_value()) << map.error();

  EXPECT_EQ(map->info.width, kColumns);
  EXPECT_EQ(map->info.height, kRows);
  EXPECT_EQ(map->data, everyCell());
}

/**
 * @brief Where the boundary between the three answers is put.
 * @details The halfway grey is a cell nobody has seen at the defaults, and a
 * description is free to move the boundary until it is a wall. What this asserts
 * is that the two thresholds are read from the file rather than compiled in.
 */
TEST_F(MapFile, TheThresholdsTheDescriptionStatesAreTheOnesApplied)
{
  writePicture();

  const std::filesystem::path description = write(kDescriptionName,
    everyRequiredKey() + "occupied_thresh: 0.4\nfree_thresh: 0.1\n");

  const std::expected<nav_msgs::msg::OccupancyGrid, std::string> map =
    mapFromFile(description.string());

  ASSERT_TRUE(map.has_value()) << map.error();

  const std::vector<std::int8_t> expected{
    kFree, kBlocked, kBlocked,
    kBlocked, kFree, kBlocked};

  EXPECT_EQ(map->data, expected);
}
