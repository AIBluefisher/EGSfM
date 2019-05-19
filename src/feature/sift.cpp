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

#include "sift.h"

#include <iostream>

namespace GraphSfM {
namespace feature {

Sift::Sift(const unsigned char image[], 
           const int width, 
           const int height,
           const int octaves, 
           const int levels, 
           const int omin)
{
    _image_data = new vl_sift_pix[width * height];
    for (int i = 0; i < width * height; i++) { _image_data[i] = image[i]; }

    _filt = vl_sift_new(width, height, octaves, levels, omin);
}

Sift::~Sift()
{
    delete _image_data;
    _image_data = nullptr;

    if (_filt) { vl_sift_delete(_filt); _filt = nullptr; }
}

bool Sift::ConfigureSift(bool compute_orientations, 
                         bool compute_descriptors, 
                         int edge_thresh, 
                         int peak_thresh, 
                         int magnif)
{   
    _force_orientations = compute_orientations;
    _force_descriptors = compute_descriptors;
    if (edge_thresh >= 0) vl_sift_set_edge_thresh(_filt, edge_thresh);
    if (peak_thresh >= 0) vl_sift_set_peak_thresh(_filt, peak_thresh);
    if (magnif >= 0)  vl_sift_set_magnif(_filt, magnif);
    return true;
}

void Sift::ShowSiftSettings()
{
    std::cout << "SIFT - Filter Settings:\n";
    std::cout << "SIFT - Octaves:              " << vl_sift_get_noctaves(_filt)          << "\n";
    std::cout << "SIFT - Levels:               " << vl_sift_get_nlevels(_filt)           << "\n";
    std::cout << "SIFT - First Octave (o_min): " << vl_sift_get_octave_first(_filt)      << "\n";
    std::cout << "SIFT - Edge Threshold:       " << vl_sift_get_edge_thresh(_filt)       << "\n";
    std::cout << "SIFT - Peak Threshold:       " << vl_sift_get_peak_thresh(_filt)       << "\n";
    std::cout << "SIFT - Magnification:        " << vl_sift_get_magnif(_filt)            << "\n";
    std::cout << "SIFT - Force Orientations:   " << (_force_orientations ? "YES" : "NO") << "\n";
    std::cout << "SIFT - Force Descriptors:    " << (_force_descriptors ? "YES" : "NO")  << "\n";
}

bool Sift::Extract(std::vector<SiftFeature>& sift_features)
{
    // Process the first octave
    if (vl_sift_process_first_octave(_filt, _image_data) == VL_ERR_EOF) {
        std::cout << "Failed when processing the first octave\n";
        return false;
    }

    while (true) {
        vl_sift_detect(_filt);

        const VlSiftKeypoint* keypoints = vl_sift_get_keypoints(_filt);
        int keypoints_num = vl_sift_get_nkeypoints(_filt);
        std::cout << "SIFT: Detected " << keypoints_num << "(unoriented) keypoints\n";

        // calculate orientation for each keypoint
        for (int i = 0; i < keypoints_num; i++) {
            double angles[4];
            int angles_num;
            VlSiftKeypoint ik;
            VlSiftKeypoint const *k;
            

            // TODO: process ikeys (not necessary)
            double *ikeys = nullptr;
            if (ikeys) {
                vl_sift_keypoint_init(_filt, &ik,
                                    ikeys[4 * i + 0],
                                    ikeys[4 * i + 1],
                                    ikeys[4 * i + 2]);
                if (ik.o != vl_sift_get_octave_index(_filt)) { break; }

                k = &ik;

                // Optionally compute orientations too
                if (_force_orientations) {
                    angles_num = vl_sift_calc_keypoint_orientations(_filt, angles, k);
                } else {
                    angles[0] = ikeys[4 * i + 3];
                    angles_num = 1;
                }
            } else {
                k = keypoints + i;
                angles_num = vl_sift_calc_keypoint_orientations(_filt, angles, k);
            }

            // for each orientation
            for (unsigned int q = 0; q < (unsigned)angles_num; q++) {
                GraphSfM::feature::SiftFeature sift_feature;
                sift_feature.SetAngle(angles[q]);
                vl_sift_pix descriptor[128];
                // Optionally: compute descriptor
                if (_force_descriptors) {
                    vl_sift_calc_keypoint_descriptor(_filt, descriptor, k, angles[q]);
                    sift_feature.SetDescriptor(descriptor);
                }
                sift_feature.SetLocation(k->x, k->y);
                sift_feature.SetScale(k->sigma);
                sift_features.push_back(sift_feature);
            }
        }   // end of outer for loop

        // Process the next octave
        if (vl_sift_process_next_octave(_filt) == VL_ERR_EOF) { break; }
    }
    return true;
}

}   // namespace feature
}   // namespace GraphSfM