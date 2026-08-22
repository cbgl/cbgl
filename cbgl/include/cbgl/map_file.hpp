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

#ifndef CBGL__MAP_FILE_HPP_
#define CBGL__MAP_FILE_HPP_

#include <expected>
#include <string>

#include <nav_msgs/msg/occupancy_grid.hpp>

namespace cbgl
{
/**
 * @brief The map a map description file on disk describes.
 * @details
 * The file is a YAML document in the format a map server reads: `image`,
 * `resolution` and `origin` are required, `negate`, `occupied_thresh` and
 * `free_thresh` are optional and default to 0, 0.65 and 0.196, and anything
 * else in it is ignored. A relative `image` is resolved against the directory
 * the description itself sits in, an absolute one is taken as it stands.
 * The image is a PGM, either the binary `P5` variant or the plain `P2` one, and
 * nothing else. Neither this package nor its ROS 1 counterpart carries an image
 * decoder the two of them could share, and PGM is what ROS map saving emits by
 * default, so the one format both builds can be held to is the one format
 * either of them reads. A picture is read on its own terms: the maximum sample
 * value it declares is what its samples are rescaled from, and a maximum above
 * 255 is two bytes a sample with the more significant byte first.
 * The grid that comes back carries no frame and no stamp. What frame a map is
 * expressed in is a setting of whoever asked for the map, not a fact the
 * description file states, so the caller labels it.
 * @return the map, or a sentence naming the file and what about it could not be
 * read. Every failure is a refusal to guess: a description missing a key, an
 * image in a format this cannot decode and a file that is not there all come
 * back as words rather than as a map with a default in place of the part that
 * was missing.
 */
std::expected<nav_msgs::msg::OccupancyGrid, std::string> mapFromFile(
  const std::string& path);

}

#endif
