// Copyright (C) 2017 Lukin Labs.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above
//       copyright notice, this list of conditions and the following
//       disclaimer in the documentation and/or other materials provided
//       with the distribution.
//
//     * Neither the name of Lukin Labs nor the names of its contributors
//       may be used to endorse or promote products derived from this software
//       without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Please contact the author of this library if you have any questions.
// Author: Maciej Janeczek (maciejjaneczek92@gmail.com)

#include <glog/logging.h>
#include <theia/theia.h>
#include "build_continuous_reconstruction_params.h"

using namespace std;
using namespace theia;

vector<string> getImages() {
    vector<string> image_files;
    CHECK(GetFilepathsFromWildcard(FLAGS_images, &image_files))
    << "Could not find images that matched the filepath: " << FLAGS_images
    << ". NOTE that the ~ filepath is not supported.";
    CHECK_GT(image_files.size(), 0) << "No images found in: " << FLAGS_images;
    sort(image_files.begin(), image_files.end());
    return image_files;
}

unordered_map<string, CameraIntrinsicsPrior> getCalibration() {
    unordered_map<std::string, CameraIntrinsicsPrior> camera_intrinsics_prior;
    if (FLAGS_calibration_file.size() != 0) {
        CHECK(ReadCalibration(FLAGS_calibration_file, &camera_intrinsics_prior))
        << "Could not read calibration file.";
    }
    return camera_intrinsics_prior;
};

void AddImagesToReconstructionBuilder(ContinuousReconstructionBuilder *reconstruction_builder,
                                      vector<string> image_files,
                                      unordered_map<string, CameraIntrinsicsPrior> camera_intrinsics_prior) {

    theia::CameraIntrinsicsGroupId intrinsics_group_id = theia::kInvalidCameraIntrinsicsGroupId;
    intrinsics_group_id = 0;

    for (const string &image_file : image_files) {
        string image_filename;
        CHECK(GetFilenameFromFilepath(image_file, true, &image_filename));

        const CameraIntrinsicsPrior *image_camera_intrinsics_prior =
                FindOrNull(camera_intrinsics_prior, image_filename);
        if (image_camera_intrinsics_prior != nullptr) {
            CHECK(reconstruction_builder->AddImageWithCameraIntrinsicsPrior(
                    image_file, *image_camera_intrinsics_prior, intrinsics_group_id));
        } else {
            CHECK(reconstruction_builder->AddImage(image_file, intrinsics_group_id));
        }
    }
    // Extract and match features.
    CHECK(reconstruction_builder->ExtractAndMatchFeatures());
}

vector<string> getNextImages(vector<string> file_names, vector<string>::iterator &it, int amount) {
    vector<string> new_file_names(it, it + amount);
    it += amount;
    return new_file_names;
}

int main(int argc, char *argv[]) {
    THEIA_GFLAGS_NAMESPACE::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);

    CHECK_GT(FLAGS_output_reconstruction.size(), 0)
        << "Must specify a filepath to output the reconstruction.";

    const ContinuousReconstructionBuilderOptions options =
            SetReconstructionBuilderOptions();

    vector<string> all_files = getImages();
    vector<string>::iterator it = all_files.begin();
    unordered_map<string, CameraIntrinsicsPrior> calibration = getCalibration();

    ContinuousReconstructionBuilder reconstruction_builder(options);

    vector<string> images = getNextImages(all_files, it, 5);
    AddImagesToReconstructionBuilder(&reconstruction_builder, images, calibration);

    std::vector<Reconstruction *> reconstructions;
    CHECK(reconstruction_builder.BuildReconstruction(&reconstructions))
    << "Could not create a reconstruction.";

    for (int i = 0; i < reconstructions.size(); i++) {
        const std::string output_file =
                theia::StringPrintf("%s-%d", FLAGS_output_reconstruction.c_str(), i);
        LOG(INFO) << "Writing reconstruction " << i << " to " << output_file;
        CHECK(theia::WriteReconstruction(*reconstructions[i], output_file))
        << "Could not write reconstruction to file.";
    }
}
