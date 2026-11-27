# Performance

This page describes what the map costs and what it bounds, and which settings govern execution time. The settings themselves are in [Parameters](../README.md#parameters) in the README.

## What the map costs and what it bounds

The map, not the algorithm, sets the floor under the accuracy. A cast ray stops at a cell, so the search matches against the room to the nearest cell, not against the room itself. That floor is at a fraction of a cell, not at a whole cell. `fsm_localisation` measures one correction against a map at about a fifteenth of a cell, at five centimetre cells and at twenty centimetre cells alike. A coarser map raises the floor, but it does not raise it to the cell size.

A finer map does not draw more hypotheses. `dl` is a density per square metre of free space. The free area is the number of free cells times the square of the cell size. So halving the cell size leaves the number of hypotheses where it was. A finer map costs the memory that the grid takes and the once per map work to prepare the grid. Both grow with the number of cells.

The map also bounds where the search may look. Every hypothesis is drawn on a cell that the map calls free. So a map whose free space is wrong in a corner cannot produce an answer in that corner. A map whose free space is a small part of its extent costs no more to sample than a full one.

## What governs execution time

`size_scan` and `top_k_caers` are the two settings to lower when a run takes too long. In the room where the timings were measured, lowering `size_scan` saved more, most of all on the first call after a map arrives. [The two settings to lower](#the-two-settings-to-lower) gives the figures.

The figures below were measured through the service, in the container. The map was one generated room with about five and a half square metres of free space. The first call after a map arrives does some work once, and every later call reuses that work. So the table gives both.

| Configuration | Threads | Candidates scored | First call | Later calls |
| --- | --- | --- | --- | --- |
| the shipped settings | all | 2,656 | 0.35 s | 0.05 s |
| ten times the hypothesis density | one | 26,688 | 0.56 s | 0.21 s |
| ten times the density, fifty times the survivors | one | 26,688 | 5.98 s | 3.19 s |
| ten times the density, one hundred and fifty times the survivors | one | 26,688 | 9.46 s | 8.85 s |

The first row differs from the second in its thread count as well as in its density. So read the three one thread rows against each other. Ten times as many candidates costs a fifth of a second. Fifty times as many survivors costs three seconds. The **refinement dominates**, not the sift. So the number of candidates that the size of a map implies matters much less than how many of them are refined.

### The two settings to lower

The table below was measured on 27 November 2026 in the same room, through the service, in the container, with all threads. The scan covered a full turn with 360 rays. Each row is one first call and 24 later calls, and the later calls give their median. Every one of the 150 calls placed the robot within 2 cm of the truth.

| `size_scan` | `top_k_caers` | First call | Later calls |
| --- | --- | --- | --- |
| `0`, all 360 rays, as shipped | `10`, as shipped | 359 ms | 54.5 ms |
| `180` | `10` | 161 ms | 36 ms |
| `90` | `10` | 44 ms | 23.5 ms |
| `0` | `5` | 326 ms | 30.5 ms |
| `0` | `2` | 356 ms | 28.5 ms |
| `90` | `2` | 37 ms | 8 ms |

Lowering `size_scan` makes the first call after a map arrives quicker, by about eight times at a quarter of the rays. Lowering `top_k_caers` leaves the first call where it was. On later calls, each setting roughly halves the time, and the two together cut it to about a seventh. The rows above refine few survivors. When many survivors are refined, as in the earlier table, the refinement dominates and `top_k_caers` matters most. Whether a lower `size_scan` costs accuracy on a sensor that does not see all the way round is not measured.

### Scope of these figures

These figures come from that one small room and from nothing larger. The number of hypotheses grows with the free area of the map, so a large building at `dl: 15.0` draws hundreds of thousands of candidates. Whether the concurrency of the sift holds up at that scale is not measured. Nothing in this documentation should be read as saying that it is.
