# Why a run is refused

This page lists every reason for a refused run. [Troubleshooting](../README.md#troubleshooting) in the README covers the most common ones and what to check for each.

## Every reason

A refused run publishes no pose. The service response gives each reason as a sentence. The second column says which part of the software decides the refusal.

| Reason | Decided by |
| --- | --- |
| no map has arrived | the node |
| no scan has arrived | the node |
| the sensor's placement is not in the transform tree | the node |
| a run is already under way | the node |
| the scan is not centred on the sensor's heading, spans more than a full turn, has an `angle_increment` that is not positive, or has an `angle_min` or `angle_increment` that is not finite | the node |
| the map has no free space at all | the algorithm |
| an empty hypothesis set was handed in | the algorithm |
| the scan's aperture is not above nought or not finite, or its `angle_min` is not finite: a scan carrying no rays at all, or the library called without the node | the algorithm |
| the scan is shorter than `size_scan` | the algorithm |
| the scan carries no usable range | the algorithm |
| every survivor was refused by the matcher | the algorithm, naming how many were refused for each reason |
| no corrected pose could be scored | the algorithm |
| the run did not finish, and what stopped it | anywhere. This reason exists for a run that runs out of memory on a large map, which then answers with a sentence instead of falling silent |

## The scan is not centred

The algorithm reads the map over one turn that begins half a turn behind the sensor's heading. It compares only the middle of that turn, as wide as the scan. So the scan must begin half its own span behind the heading. A valid scan that begins more than one ray away from that angle would give an answer that is rotated. The node refuses such a scan, and names the field and the value that it carried, instead of answering wrongly.

A scan narrower than a full turn is not refused for being narrower.
