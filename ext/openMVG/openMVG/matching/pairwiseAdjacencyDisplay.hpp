// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2012, 2013 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef OPENMVG_PAIRWISE_ADJACENCY_DISPLAY_HPP
#define OPENMVG_PAIRWISE_ADJACENCY_DISPLAY_HPP

#include <string>

#include "openMVG/matching/indMatch.hpp"

#include "third_party/vectorGraphics/svgDrawer.hpp"

namespace openMVG  {
namespace matching {

/// Display pair wises matches as an Adjacency matrix in svg format
void PairWiseMatchingToAdjacencyMatrixSVG
(
  const size_t NbImages,
  const matching::PairWiseMatches & map_Matches,
  const std::string & sOutName
);

} // namespace matching
} // namespace openMVG

#endif // OPENMVG_PAIRWISE_ADJACENCY_DISPLAY_HPP
