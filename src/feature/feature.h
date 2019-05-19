// Copyright (c) 2018, 陈煜
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.

// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.

// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef FEATURE_FEATURE_H
#define FEATURE_FEATURE_H

#include <vector>

extern "C" {
    #include <vlfeat/generic.h>
    #include <vlfeat/stringop.h>
    #include <vlfeat/pgm.h>
    #include <vlfeat/sift.h>
    #include <vlfeat/getopt_long.h>
};

namespace GraphSfM {
namespace feature {

class FeatureBase
{
private:
    double _x;
    double _y;

public:
    FeatureBase() { }
    FeatureBase(int x, int y) { _x = x; _y = y; }

    void SetX(double x) { _x = x; }
    void SetY(double y) { _y = y; }
    void SetLocation(double x, double y) { _x = x; _y = y; }

    double GetX() const { return _x; };
    double GetY() const { return _y; };
};

class SiftFeature : public FeatureBase
{
private:
    double _scale;  // the radius of the region
    /**
     * @brief  angles expressed in radians
     * @note   Depending on the symmetry of the keypoint appearance, 
     * determining the orientation can be ambiguous. In this case, the SIFT detectors
     * returns a list of up to four possible orientations, constructing up to four
     * frames (differing only by their orientation) for each detected image blob.
     */
    double _angles;
    vl_sift_pix* _descriptors;

public:
    SiftFeature() { }
    SiftFeature(int x, int y, int s) : FeatureBase(x, y) { _scale = s; }

    void SetScale(double s) { _scale = s; }
    void SetAngle(double angle) { _angles = angle; }
    void SetDescriptor(vl_sift_pix* des) { _descriptors = des; }

    double GetScale() const { return _scale; }
    double GetAngles() const { return _angles; }
    vl_sift_pix* GetDescriptors() const { return _descriptors; }
};

}   // namespace feature
}   // namespace GraphSfM

#endif