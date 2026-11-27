# Licensing

This page gives the licence of the sources of `cbgl` and the obligations that a redistributed binary carries. [Licensing](../README.md#licensing) in the README gives the summary.

## The sources

The own sources of `cbgl` are MIT. See [LICENSE](../LICENSE). Nothing below changes that, because MIT code may link a library under any of these terms.

## A redistributed binary

A **redistributed binary** is a different question. It carries obligations from three libraries, all of them through `fsm_localisation`.

| From | Through | Terms |
| --- | --- | --- |
| CGAL | `fsm_localisation` | GPL v3 |
| FFTW | `fsm_localisation` | GPL v2 or later |
| the ray casting inside `fsm_localisation` | `fsm_localisation` | Apache 2.0 |

The terms of FFTW allow version 3, so the two GPL rows combine to GPL v3 and do not conflict. The GPL v3 text travels with the work, as that licence asks. It is in [COPYING](../COPYING), and it is installed into `share/cbgl_global_localisation` so that it reaches a built tree and not only these sources.

## Obligations that are gone

The ROS 1 version also carried LGPL v2.1 and Apache 2.0 obligations. They came from the map and particle filter of amcl and from a vendored ray casting library. Both are gone from these sources, together with the code that brought them.

**This version briefly carried an LGPL v3 obligation as well, through Andrea Censi's canonical scan matcher, and no longer does.** That refinement was removed after it was measured against the scan matcher at a full turn and at three quarters of a turn. [Refinement](parameters.md#refinement) gives the figures. `COPYING.LESSER` went with it. If you compare against a state of this repository that still names it, that page also explains why it went.

## Not legal advice

This is a reading of the licences and not legal advice. If a binary is ever redistributed commercially, it needs a real review by somebody qualified to give one.
