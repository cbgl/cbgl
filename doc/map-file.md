# Map files

This page describes the map description file that `map_file` names, and how the node reads its image. [Taking the map from a file instead of a topic](../README.md#taking-the-map-from-a-file-instead-of-a-topic) in the README shows how to select it.

## The description file

The description file has the format that a map server reads:

```yaml
image: room.pgm
resolution: 0.05
origin: [-1.35, 2.70, 0.60]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

| Key | Required | Default when absent |
| --- | --- | --- |
| `image` | yes | none |
| `resolution` | yes | none |
| `origin` | yes | none |
| `negate` | no | `0` |
| `occupied_thresh` | no | `0.65` |
| `free_thresh` | no | `0.196` |

The node ignores any other key in the file. A relative image path is resolved against the directory that the description file is in.

## The image

The image is a **PGM**, either the binary `P5` variant or the plain `P2` variant, which is what ROS map saving writes. The node reads no other format. This package links no image library, and neither does its ROS 1 counterpart. So PGM is the one format that both of them can be held to.

The node reads each image on its own terms. It rescales the samples from the maximum sample value that the image declares. So in an image that declares 15, a sample of 15 is white, exactly as 255 is white in a saved map. A maximum above 255 means two bytes a sample, with the more significant byte first. The node refuses three cases by name. They are a maximum of nought, a maximum above 65535, and a sample above the maximum that its own image declares.

## When the file is read

The node reads the file once, at start-up. The map that the file describes is then the map for the life of the node.

Four faults stop the node at start-up: a description that cannot be read, a key that is missing, an image in another format, and a map that nothing can be localised in. The refusal names the file and the key. So a bad file never gives a node that comes up and then refuses every request.

If both `map_topic` and `map_file` name something, the topic wins, the node ignores the file, and a warning says so. If neither names something, the node refuses to start.
