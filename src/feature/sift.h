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

#ifndef FEATURE_SIFT_H
#define FEATURE_SIFT_H

#include <vector>
#include "feature.h"

extern "C" {
    #include <vlfeat/generic.h>
    #include <vlfeat/stringop.h>
    #include <vlfeat/pgm.h>
    #include <vlfeat/sift.h>
    #include <vlfeat/getopt_long.h>
};

namespace GraphSfM {
namespace feature {

class Sift
{
private:
    vl_sift_pix* _image_data;
    VlSiftFilt* _filt;

    bool _force_orientations;   // Determin whether computing orientations
    bool _force_descriptors;    // Determin whether computing descriptors

public:
    Sift(const unsigned char image[], 
         const int width, 
         const int height,
         const int octaves, 
         const int levels, 
         const int omin);
    ~Sift();

    bool ConfigureSift(bool compute_orientations = true, 
                       bool compute_descriptors = false, 
                       int edge_thresh = -1, 
                       int peak_thresh = -1, 
                       int magnif = -1);


    bool Extract(std::vector<SiftFeature>& sift_features);
    

    void ShowSiftSettings();
};

}   // namespace feature
}   // namespace GraphSfM

#endif