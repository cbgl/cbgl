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
 * @brief `cbgl::mapFromFile`.
 */
#include "cbgl/map_file.hpp"

#include <array>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <format>
#include <fstream>
#include <ios>
#include <iterator>
#include <optional>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace cbgl
{
namespace
{
const std::int8_t kBlocked = 100;
const std::int8_t kFree = 0;
const std::int8_t kUnknown = -1;

const double kDefaultFreeThreshold = 0.196;
const double kDefaultOccupiedThreshold = 0.65;

/**
 * @brief The largest sample value a PGM may declare, and the largest one that
 * fits in a single byte.
 * @details A PGM states what its own white is, and the value it states decides
 * how wide a sample is on disk as well: one byte up to 255 and two bytes above
 * it, the more significant first. A map saved by ROS declares 255 and is
 * therefore one byte a sample, and a picture declaring anything else is read on
 * its own terms rather than refused.
 */
const unsigned int kLargestOneByteSample = 255U;
const unsigned int kLargestSample = 65535U;

/** @brief The largest map image this reads, in bytes. Two bytes a cell over a
 *  grid far larger than any a robot carries, and small enough that refusing
 *  anything bigger costs nobody anything. */
const std::uintmax_t kLargestImageBytes = 1200000000U;

/**
 * @brief A greyscale image as a PGM carries one: one sample per pixel, the
 * first row being the top of the picture.
 * @details The samples are on nought to 255 whatever the picture declared its
 * own maximum to be, having been rescaled onto that range as they were read.
 */
struct Image
{
  unsigned int width{0U};
  unsigned int height{0U};
  std::vector<unsigned char> samples;
};

/**
 * @brief A sample rescaled onto one byte from whatever maximum its own picture
 * declared.
 * @details A PGM states the value its white is, and 255 is only the commonest
 * choice: a picture declaring 15 means its 15 to be white exactly as a picture
 * declaring 255 means its 255. Every sample is put on the nought to 255 scale
 * the occupancy rule reads, here and nowhere else. Read unscaled, a picture
 * declaring a small maximum comes back almost entirely blocked, with nothing
 * said about it and nothing downstream able to notice.
 * @remark `sample` must not exceed `maximum`. Every caller refuses a picture
 * carrying a sample above the maximum that same picture states, so the result
 * cannot leave the range one byte holds.
 */
unsigned char eightBitSample(const unsigned int sample,
  const unsigned int maximum)
{
  return static_cast<unsigned char>(static_cast<double>(sample) * 255.0
    / static_cast<double>(maximum) + 0.5);
}

bool isSpace(const unsigned char byte)
{
  return byte == ' ' || byte == '\t' || byte == '\n' || byte == '\r'
    || byte == '\v' || byte == '\f';
}

/**
 * @brief The next whitespace separated token of a PGM, comments skipped.
 * @details A comment runs from a `#` to the end of its line and is allowed
 * wherever whitespace is, which is why this is not a plain split on spaces.
 * @return the token, or nothing at all where the file ends before one begins.
 */
std::optional<std::string> nextToken(const std::vector<unsigned char>& bytes,
  std::size_t& at)
{
  while (at < bytes.size())
  {
    if (isSpace(bytes[at]))
    {
      at++;
      continue;
    }

    if (bytes[at] == '#')
    {
      while (at < bytes.size() && bytes[at] != '\n')
        at++;

      continue;
    }

    break;
  }

  if (at >= bytes.size())
    return std::nullopt;

  const std::size_t from = at;

  while (at < bytes.size() && !isSpace(bytes[at]) && bytes[at] != '#')
    at++;

  return std::string(bytes.begin() + static_cast<std::ptrdiff_t>(from),
    bytes.begin() + static_cast<std::ptrdiff_t>(at));
}

/**
 * @brief The number a key of a map description carries.
 * @details The key is required: a description that does not carry it is a
 * description of a map nobody can place, and a default put in its stead would
 * be this package inventing half a map.
 */
std::expected<double, std::string> numberAt(const YAML::Node& description,
  const std::string& key, const std::string& path)
{
  const YAML::Node value = description[key];

  if (!value)
  {
    return std::unexpected(std::format(
      "the map description {} carries no {}, which is required", path, key));
  }

  try
  {
    return value.as<double>();
  }
  catch (const YAML::Exception& exception)
  {
    return std::unexpected(std::format(
      "the map description {} carries a {} that is not a number: {}", path, key,
      exception.what()));
  }
}

/**
 * @brief The number a key of a map description carries, or `fallback` where the
 * description does not carry that key at all.
 * @details A key that is there and is not a number is still a refusal. Reading
 * it as the fallback would turn a typed value into silence.
 */
std::expected<double, std::string> numberAt(const YAML::Node& description,
  const std::string& key, const std::string& path, const double fallback)
{
  if (!description[key])
    return fallback;

  return numberAt(description, key, path);
}

/**
 * @brief What an image's samples amount to on the nought to one hundred scale
 * an occupancy grid states a cell in.
 * @details The conversion is the map server's own trinary rule: a sample
 * becomes an occupancy of `(255 - sample) / 255`, or of `sample / 255` where the
 * description asks for the image to be negated, and that occupancy is then read
 * as blocked, as free, or as a cell nobody has seen. The samples are already on
 * nought to 255, whatever the picture declared, the decoder having put them
 * there.
 * @remark The rows come back to front, and they have to. A PGM's first row is
 * the top of the picture and an occupancy grid's first row is the one at the
 * origin, which is the bottom of it, so a map read in the order its file
 * carries is that map reflected. Nothing downstream can notice: the reflection
 * is a legal map, of a building that does not exist, and every answer found in
 * it is confidently wrong.
 */
std::vector<std::int8_t> occupancyOf(const Image& image, const bool negate,
  const double occupied_threshold, const double free_threshold)
{
  std::vector<std::int8_t> cells(
    static_cast<std::size_t>(image.width) * image.height, kUnknown);

  for (unsigned int row = 0U; row < image.height; row++)
    for (unsigned int column = 0U; column < image.width; column++)
    {
      const std::size_t read_at =
        static_cast<std::size_t>(image.height - 1U - row) * image.width
        + column;
      const double sample = image.samples[read_at];
      const double occupancy = negate
        ? sample / 255.0 : (255.0 - sample) / 255.0;

      const std::size_t write_at =
        static_cast<std::size_t>(row) * image.width + column;

      if (occupancy > occupied_threshold)
        cells[write_at] = kBlocked;
      else if (occupancy < free_threshold)
        cells[write_at] = kFree;
    }

  return cells;
}

/**
 * @brief Where a map description places the grid's first cell: x and y in
 * metres and a yaw in radians.
 */
std::expected<std::array<double, 3>, std::string> originAt(
  const YAML::Node& description, const std::string& path)
{
  const YAML::Node value = description["origin"];

  if (!value)
  {
    return std::unexpected(std::format(
      "the map description {} carries no origin, which is required", path));
  }

  if (!value.IsSequence() || value.size() != 3)
  {
    return std::unexpected(std::format(
      "the map description {} carries an origin of {} values, where an origin "
      "is three: x and y in metres and a yaw in radians", path, value.size()));
  }

  std::array<double, 3> origin{};

  for (std::size_t element = 0; element < origin.size(); element++)
  {
    try
    {
      origin[element] = value[element].as<double>();
    }
    catch (const YAML::Exception& exception)
    {
      return std::unexpected(std::format(
        "the map description {} carries an origin whose value {} is not a "
        "number: {}", path, element, exception.what()));
    }

    if (!std::isfinite(origin[element]))
    {
      return std::unexpected(std::format(
        "the map description {} carries an origin whose value {} is {}, and an "
        "origin must be finite", path, element, origin[element]));
    }
  }

  return origin;
}

/**
 * @brief The picture a PGM's bytes carry.
 * @details The magic number is read from the first two bytes rather than as a
 * token, so that a file which is not a PGM at all is named as one thing this
 * cannot decode instead of being scanned for whitespace it may not carry.
 * The maximum the picture declares decides two things and both are honoured
 * here: how wide a sample is, one byte up to 255 and two above it with the more
 * significant first, and what scale the samples are on, which is why every one
 * of them is rescaled onto a byte as it is read.
 * @remark A sample above the maximum its own picture states is refused rather
 * than rescaled. Rescaling one would put it past what a byte holds, and a
 * picture that contradicts its own header is not a picture to guess at.
 * @return the picture, or a sentence naming the file and what about its bytes
 * could not be read as a PGM.
 */
std::expected<Image, std::string> pgmOf(const std::vector<unsigned char>& bytes,
  const std::string& path)
{
  if (bytes.size() < 2 || bytes[0] != 'P'
    || (bytes[1] != '2' && bytes[1] != '5'))
  {
    return std::unexpected(std::format(
      "the map image {} is not a PGM. This package decodes the binary \"P5\" "
      "and the plain \"P2\" variants of PGM and no other image format at all",
      path));
  }

  const bool binary = bytes[1] == '5';
  std::size_t at = 2;

  const auto whole = [&bytes, &at]() -> std::optional<unsigned int>
    {
      const std::optional<std::string> token = nextToken(bytes, at);

      if (!token.has_value())
        return std::nullopt;

      unsigned int value = 0U;
      const char* const from = token->data();
      const char* const to = from + token->size();
      const std::from_chars_result read = std::from_chars(from, to, value);

      if (read.ec != std::errc() || read.ptr != to)
        return std::nullopt;

      return value;
    };

  const std::optional<unsigned int> width = whole();
  const std::optional<unsigned int> height = whole();
  const std::optional<unsigned int> largest = whole();

  if (!width.has_value() || !height.has_value() || !largest.has_value())
  {
    return std::unexpected(std::format(
      "the map image {} carries no width, height and maximum sample value that "
      "can be read as whole numbers", path));
  }

  if (*width == 0U || *height == 0U)
  {
    return std::unexpected(std::format(
      "the map image {} states a size of {} by {} pixels, and a map must carry "
      "a cell", path, *width, *height));
  }

  if (*largest == 0U || *largest > kLargestSample)
  {
    return std::unexpected(std::format(
      "the map image {} states a maximum sample value of {}, and a PGM's "
      "maximum is between 1 and {}", path, *largest, kLargestSample));
  }

  const std::size_t pixels = static_cast<std::size_t>(*width) * *height;
  const std::size_t bytes_a_sample = *largest > kLargestOneByteSample ? 2U : 1U;

  /* Nothing is sized from the header until the header has been measured
   * against the bytes actually in hand. A picture states its own dimensions
   * and nothing in it obliges them to describe what follows, so reserving
   * from them first turns a mistyped digit into an allocation of every byte
   * the machine has, and a product that wraps turns it into an index that
   * walks outside the buffer. Both are ruled out here rather than caught
   * afterwards, and this must stay above the reserve below. A binary sample
   * takes at least one byte on disk and a text one at least two counting its
   * separator, so either way there is a floor to check against. */
  const std::size_t after_header = bytes.size() - at;
  const std::size_t floor_a_sample = binary ? bytes_a_sample : 2U;

  if (pixels > after_header / floor_a_sample)
  {
    return std::unexpected(std::format(
      "the map image {} states a size of {} by {} pixels, which need at least "
      "{} bytes each, and carries {} after its header altogether", path,
      *width, *height, floor_a_sample, after_header));
  }

  Image image{.width = *width, .height = *height, .samples = {}};
  image.samples.reserve(pixels);

  if (binary)
  {
    /* One whitespace byte is what the format calls for. A picture carrying
     * two, which is what a CRLF separator gives and what any pass through a
     * Windows editor produces, would otherwise hand that second byte over as
     * the first sample, shift every pixel by one and drop the last, with the
     * check below still satisfied because the file holds one byte more than
     * it needs. That is a map wrong everywhere and silent, so the separator is
     * consumed exactly rather than approximately. */
    if (at < bytes.size() && bytes[at] == '\r' && at + 1U < bytes.size()
      && bytes[at + 1U] == '\n')
      at += 2U;
    else if (at < bytes.size() && isSpace(bytes[at]))
      at++;

    const std::size_t needed = pixels * bytes_a_sample;

    if (bytes.size() - at < needed)
    {
      return std::unexpected(std::format(
        "the map image {} states a size of {} by {} pixels at {} bytes a "
        "sample, which needs {} bytes, and carries {} after its header", path,
        *width, *height, bytes_a_sample, needed, bytes.size() - at));
    }

    for (std::size_t pixel = 0; pixel < pixels; pixel++)
    {
      const std::size_t read_at = at + pixel * bytes_a_sample;
      const unsigned int sample = bytes_a_sample == 2
        ? static_cast<unsigned int>(bytes[read_at]) * 256U + bytes[read_at + 1]
        : bytes[read_at];

      if (sample > *largest)
      {
        return std::unexpected(std::format(
          "the map image {} carries the sample {}, which is above the maximum "
          "of {} the image itself states", path, sample, *largest));
      }

      image.samples.push_back(eightBitSample(sample, *largest));
    }

    return image;
  }

  while (image.samples.size() < pixels)
  {
    const std::optional<unsigned int> sample = whole();

    if (!sample.has_value())
    {
      return std::unexpected(std::format(
        "the map image {} states a size of {} by {} pixels and carries {} of "
        "the {} samples that size needs", path, *width, *height,
        image.samples.size(), pixels));
    }

    if (*sample > *largest)
    {
      return std::unexpected(std::format(
        "the map image {} carries the sample {}, which is above the maximum of "
        "{} the image itself states", path, *sample, *largest));
    }

    image.samples.push_back(eightBitSample(*sample, *largest));
  }

  return image;
}

/**
 * @brief The picture a file holds.
 * @details The whole file is taken into memory before a byte of it is read. A
 * map is read once, at start up, and the alternative is a decoder that has to
 * be correct about a stream as well as about a format.
 */
std::expected<Image, std::string> readImage(const std::filesystem::path& path)
{
  /* What the path names is established before it is opened. A path naming a
   * character device reads until the memory runs out; one naming a pipe blocks
   * here for ever, and a node that never finishes starting and never says why
   * is the worst outcome on offer. Both are ruled out by insisting on an
   * ordinary file of a sane size. */
  std::error_code code;
  const std::filesystem::file_status status =
    std::filesystem::status(path, code);

  if (code || !std::filesystem::is_regular_file(status))
  {
    return std::unexpected(std::format(
      "the map image {} is not an ordinary file, and this reads a PGM from a "
      "file rather than from a device or a pipe", path.string()));
  }

  const std::uintmax_t size = std::filesystem::file_size(path, code);

  if (code)
  {
    return std::unexpected(std::format(
      "the map image {} cannot be measured", path.string()));
  }

  if (size > kLargestImageBytes)
  {
    return std::unexpected(std::format(
      "the map image {} is {} bytes, above the {} this reads", path.string(),
      size, kLargestImageBytes));
  }

  std::ifstream file(path, std::ios::binary);

  if (!file.good())
  {
    return std::unexpected(std::format(
      "the map image {} cannot be read", path.string()));
  }

  const std::vector<unsigned char> bytes{
    std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};

  return pgmOf(bytes, path.string());
}

/**
 * @brief The text a key of a map description carries.
 */
std::expected<std::string, std::string> textAt(const YAML::Node& description,
  const std::string& key, const std::string& path)
{
  const YAML::Node value = description[key];

  if (!value)
  {
    return std::unexpected(std::format(
      "the map description {} carries no {}, which is required", path, key));
  }

  try
  {
    return value.as<std::string>();
  }
  catch (const YAML::Exception& exception)
  {
    return std::unexpected(std::format(
      "the map description {} carries an {} that is not text: {}", path, key,
      exception.what()));
  }
}

/**
 * @brief The whole number a key of a map description carries, or `fallback`
 * where the description does not carry that key at all.
 */
std::expected<int, std::string> wholeNumberAt(const YAML::Node& description,
  const std::string& key, const std::string& path, const int fallback)
{
  const YAML::Node value = description[key];

  if (!value)
    return fallback;

  try
  {
    return value.as<int>();
  }
  catch (const YAML::Exception& exception)
  {
    return std::unexpected(std::format(
      "the map description {} carries a {} that is not a whole number: {}",
      path, key, exception.what()));
  }
}

}

/**
 * @details The description is read first and the image second, so a file that
 * names an image it does not have fails naming the image rather than the
 * description. Every value the grid carries comes from one of the two: the
 * resolution and the origin from the description, the width and the height from
 * the picture, and each cell from a sample of it.
 * @remark A resolution is refused here, rather than left to whoever builds a
 * map from the message, so that the refusal can name the key and the file it
 * came from. A reader of that sentence has a file to open.
 */
std::expected<nav_msgs::msg::OccupancyGrid, std::string> mapFromFile(
  const std::string& path)
{
  YAML::Node description;

  try
  {
    description = YAML::LoadFile(path);
  }
  catch (const YAML::Exception& exception)
  {
    return std::unexpected(std::format(
      "the map description {} cannot be read: {}", path, exception.what()));
  }

  if (!description.IsMap())
  {
    return std::unexpected(std::format(
      "the map description {} is not a mapping of keys to values, so it "
      "describes no map", path));
  }

  /* The map server reads `mode` as trinary, scale or raw, and the three give
   * three different maps from one picture. This reads trinary. A description
   * asking for either of the others is refused rather than quietly handed the
   * one it did not ask for. */
  if (description["mode"])
  {
    const std::expected<std::string, std::string> mode =
      textAt(description, "mode", path);

    if (!mode.has_value())
      return std::unexpected(mode.error());

    if (*mode != "trinary")
    {
      return std::unexpected(std::format(
        "the map description {} asks for the {} mode, and this reads trinary "
        "only", path, *mode));
    }
  }

  const std::expected<std::string, std::string> image =
    textAt(description, "image", path);

  if (!image.has_value())
    return std::unexpected(image.error());

  const std::expected<double, std::string> resolution =
    numberAt(description, "resolution", path);

  if (!resolution.has_value())
    return std::unexpected(resolution.error());

  if (!std::isfinite(*resolution) || *resolution <= 0.0)
  {
    return std::unexpected(std::format(
      "the map description {} carries a resolution of {}, and a resolution "
      "must be finite and greater than zero", path, *resolution));
  }

  const std::expected<std::array<double, 3>, std::string> origin =
    originAt(description, path);

  if (!origin.has_value())
    return std::unexpected(origin.error());

  const std::expected<int, std::string> negate =
    wholeNumberAt(description, "negate", path, 0);

  if (!negate.has_value())
    return std::unexpected(negate.error());

  const std::expected<double, std::string> occupied_threshold =
    numberAt(description, "occupied_thresh", path, kDefaultOccupiedThreshold);

  if (!occupied_threshold.has_value())
    return std::unexpected(occupied_threshold.error());

  const std::expected<double, std::string> free_threshold =
    numberAt(description, "free_thresh", path, kDefaultFreeThreshold);

  if (!free_threshold.has_value())
    return std::unexpected(free_threshold.error());

  /* These two decide what every cell of the map means, and until now they were
   * the only numbers in the description taken as they came. A threshold above
   * one leaves nothing blocked, so every wall becomes free and no cast meets
   * anything; one below nought makes every cell a wall; a free threshold above
   * the occupied one means no cell is ever unknown; and either as a NaN makes
   * every comparison false, so the whole map is unknown and the node refuses
   * every request afterwards for a reason that names neither the file nor the
   * key. All of them are silent, so all of them are refused here. */
  for (const auto& [name, value] : {
    std::pair<const char*, double>{"occupied_thresh", *occupied_threshold},
    std::pair<const char*, double>{"free_thresh", *free_threshold}})
  {
    if (!std::isfinite(value) || value < 0.0 || value > 1.0)
    {
      return std::unexpected(std::format(
        "the map description {} carries a {} of {}, and a threshold is a "
        "fraction between nought and one", path, name, value));
    }
  }

  if (*free_threshold > *occupied_threshold)
  {
    return std::unexpected(std::format(
      "the map description {} carries a free_thresh of {} above its "
      "occupied_thresh of {}, which would leave no cell unknown", path,
      *free_threshold, *occupied_threshold));
  }

  const std::filesystem::path named(*image);
  const std::filesystem::path image_path = named.is_absolute()
    ? named : std::filesystem::path(path).parent_path() / named;

  const std::expected<Image, std::string> picture = readImage(image_path);

  if (!picture.has_value())
    return std::unexpected(picture.error());

  nav_msgs::msg::OccupancyGrid map;
  map.info.resolution = static_cast<float>(*resolution);
  map.info.width = picture->width;
  map.info.height = picture->height;
  map.info.origin.position.x = (*origin)[0];
  map.info.origin.position.y = (*origin)[1];
  map.info.origin.position.z = 0.0;
  map.info.origin.orientation.x = 0.0;
  map.info.origin.orientation.y = 0.0;
  map.info.origin.orientation.z = std::sin((*origin)[2] / 2.0);
  map.info.origin.orientation.w = std::cos((*origin)[2] / 2.0);
  map.data = occupancyOf(*picture, *negate != 0, *occupied_threshold,
    *free_threshold);

  return map;
}

}
