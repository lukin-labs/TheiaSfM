//
// Created by maciek on 06.05.17.
//

#include "continuous_feature_extractor_and_matcher.h"

#include <Eigen/Core>
#include <glog/logging.h>
#include <algorithm>
#include <memory>
#include <string>
#include <thread>  // NOLINT
#include <vector>

#include "theia/image/image.h"
#include "theia/image/descriptor/create_descriptor_extractor.h"
#include "theia/image/descriptor/descriptor_extractor.h"
#include "theia/image/keypoint_detector/keypoint.h"
#include "theia/matching/create_feature_matcher.h"
#include "theia/matching/feature_correspondence.h"
#include "theia/matching/feature_matcher_options.h"
#include "theia/matching/image_pair_match.h"
#include "theia/sfm/camera_intrinsics_prior.h"
#include "theia/sfm/estimate_twoview_info.h"
#include "theia/sfm/exif_reader.h"
#include "theia/sfm/two_view_match_geometric_verification.h"
#include "theia/util/filesystem.h"
#include "theia/util/string.h"
#include "theia/util/threadpool.h"

namespace theia {
    namespace {

        void ExtractFeatures(
                const ContinuousFeatureExtractorAndMatcher::Options &options,
                const std::string &image_filepath,
                std::vector<Keypoint> *keypoints,
                std::vector<Eigen::VectorXf> *descriptors) {
            static const float kMaskThreshold = 0.5;
            std::unique_ptr<FloatImage> image(new FloatImage(image_filepath));
            // We create these variable here instead of upon the construction of the
            // object so that they can be thread-safe. We *should* be able to use the
            // static thread_local keywords, but apparently Mac OS-X's version of clang
            // does not actually support it!
            //
            // TODO(cmsweeney): Change this so that each thread in the threadpool receives
            // exactly one object.
            std::unique_ptr<DescriptorExtractor> descriptor_extractor =
                    CreateDescriptorExtractor(options.descriptor_extractor_type,
                                              options.feature_density);

            // Exit if the descriptor extraction fails.
            if (!descriptor_extractor->DetectAndExtractDescriptors(*image,
                                                                   keypoints,
                                                                   descriptors)) {
                LOG(ERROR) << "Could not extract descriptors in image " << image_filepath;
                return;
            }


            if (keypoints->size() > options.max_num_features) {
                keypoints->resize(options.max_num_features);
                descriptors->resize(options.max_num_features);
            }
            VLOG(1) << "Successfully extracted " << descriptors->size()
                    << " features from image " << image_filepath;

        }

    }  // namespace

    ContinuousFeatureExtractorAndMatcher::ContinuousFeatureExtractorAndMatcher(
            const ContinuousFeatureExtractorAndMatcher::Options &options)
            : options_(options) {
        // Create the feature matcher.
        FeatureMatcherOptions matcher_options = options_.feature_matcher_options;
        matcher_options.num_threads = options_.num_threads;
        matcher_options.min_num_feature_matches = options_.min_num_inlier_matches;
        matcher_options.perform_geometric_verification = true;
        matcher_options.geometric_verification_options.min_num_inlier_matches =
                options_.min_num_inlier_matches;

        matcher_ = CreateFeatureMatcher(options_.matching_strategy, matcher_options);
    }

    bool ContinuousFeatureExtractorAndMatcher::AddImage(const std::string &image_filepath) {
        image_filepaths_.emplace_back(image_filepath);
        return true;
    }

    bool ContinuousFeatureExtractorAndMatcher::AddImage(
            const std::string &image_filepath,
            const CameraIntrinsicsPrior &intrinsics) {
        if (!AddImage(image_filepath)) {
            return false;
        }
        intrinsics_[image_filepath] = intrinsics;
        return true;
    }

    bool ContinuousFeatureExtractorAndMatcher::AddMaskForFeaturesExtraction(
            const std::string &image_filepath,
            const std::string &mask_filepath) {
        image_masks_[image_filepath] = mask_filepath;
        VLOG(1) << "Image: " << image_filepath << " || "
                << "Associated mask: " << mask_filepath;
        return true;
    }

// Performs feature matching between all images provided by the image
// filepaths. Features are extracted and matched between the images according to
// the options passed in. Only matches that have passed geometric verification
// are kept. EXIF data is parsed to determine the camera intrinsics if
// available.
    void ContinuousFeatureExtractorAndMatcher::ExtractAndMatchFeatures(
            std::vector<CameraIntrinsicsPrior> *intrinsics,
            std::vector<ImagePairMatch> *matches) {
        CHECK_NOTNULL(intrinsics)->resize(image_filepaths_.size());
        CHECK_NOTNULL(matches);
        CHECK_NOTNULL(matcher_.get());

        // For each image, process the features and add it to the matcher.
        // Using only one thread to increase stability
        // In future will be replaced by CUDA implementation
        std::vector<std::string> new_files;
        const int num_threads =
                std::min(options_.num_threads, static_cast<int>(image_filepaths_.size()));
        std::unique_ptr<ThreadPool> thread_pool(new ThreadPool(num_threads));
        for (int i = 0; i < image_filepaths_.size(); i++) {
            if (!FileExists(image_filepaths_[i])) {
                LOG(ERROR) << "Could not extract features for " << image_filepaths_[i]
                           << " because the file cannot be found.";
                continue;
            }
            // Check if image is already added
            std::string image_filename;
            CHECK(GetFilenameFromFilepath(image_filepaths_[i], true, &image_filename));
            if (!matcher_->Contains(image_filename)) {
                ContinuousFeatureExtractorAndMatcher::ProcessImage(i);
                new_files.push_back(image_filepaths_[i]);
            }
        }

        // After all threads complete feature extraction, sellect pairs to be matched.
        //TODO: Select pairs of the images to be matched. Generally should be each new image with all other.
        //      In future should be matched only to nearby images.
        std::vector<std::pair<std::string, std::string>> pairs;
        std::vector<std::string>::iterator it = new_files.begin();
        for (auto new_file : new_files) {
            for (auto image_path : image_filepaths_) {
                //Check if image is different than this image and such pair was not yet generated
                if (image_path != new_file /*&& std::find(new_files.begin(), it, image_path) == it*/) {
                    // Get the image filename without the directory.
                    std::string image_filename1, image_filename2;
                    CHECK(GetFilenameFromFilepath(image_path, true, &image_filename1));
                    CHECK(GetFilenameFromFilepath(new_file, true, &image_filename2));
                    std::pair<std::string, std::string> pair(image_filename1, image_filename2);
                    pairs.push_back(pair);
                }
            }
            //it += 1;
        }

        matcher_->SetImagePairsToMatch(pairs);

        // perform matching.

        // Perform the matching.
        LOG(INFO) << "Matching images...";
        matcher_->MatchImages(matches);

        // Add the intrinsics to the output.
        for (int i = 0; i < image_filepaths_.size(); i++) {
            (*intrinsics)[i] = FindOrDie(intrinsics_, image_filepaths_[i]);
        }
    }

    void ContinuousFeatureExtractorAndMatcher::ProcessImage(
            const int i) {
        const std::string &image_filepath = image_filepaths_[i];

        // Get the camera intrinsics prior if it was provided.
        CameraIntrinsicsPrior intrinsics =
                FindWithDefault(intrinsics_, image_filepath, CameraIntrinsicsPrior());

        // Extract an EXIF focal length if it was not provided.
        if (!intrinsics.focal_length.is_set) {
            CHECK(exif_reader_.ExtractEXIFMetadata(image_filepath, &intrinsics));

            // If the focal length still could not be extracted, set it to a reasonable
            // value based on a median viewing angle.
            if (!options_.only_calibrated_views && !intrinsics.focal_length.is_set) {
                VLOG(2) << "Exif was not detected. Setting it to a reasonable value.";
                intrinsics.focal_length.is_set = true;
                intrinsics.focal_length.value[0] =
                        1.2 * static_cast<double>(
                                std::max(intrinsics.image_width, intrinsics.image_height));
            }

            std::lock_guard<std::mutex> lock(intrinsics_mutex_);
            // Insert or update the value of the intrinsics.
            intrinsics_[image_filepath] = intrinsics;
        }

        // Early exit if no EXIF calibration exists and we are only processing
        // calibration views.
        if (options_.only_calibrated_views && !intrinsics.focal_length.is_set) {
            LOG(INFO) << "Image " << image_filepath
                      << " did not contain an EXIF focal length. Skipping this image.";
            return;
        } else {
            LOG(INFO) << "Image " << image_filepath
                      << " is initialized with the focal length: "
                      << intrinsics.focal_length.value[0];
        }

        // Get the image filename without the directory.
        std::string image_filename;
        CHECK(GetFilenameFromFilepath(image_filepath, true, &image_filename));

        // Get the feature filepath based on the image filename.
        std::string output_dir =
                options_.feature_matcher_options.keypoints_and_descriptors_output_dir;
        AppendTrailingSlashIfNeeded(&output_dir);
        const std::string feature_filepath =
                output_dir + image_filename + ".features";

        // If the feature file already exists, skip the feature extraction.
        if (options_.feature_matcher_options.match_out_of_core &&
            FileExists(feature_filepath)) {
            std::lock_guard<std::mutex> lock(matcher_mutex_);
            matcher_->AddImage(image_filename, intrinsics);
            return;
        }

        // Extract Features.
        std::vector<Keypoint> keypoints;
        std::vector<Eigen::VectorXf> descriptors;
        ExtractFeatures(options_,
                        image_filepath,
                        &keypoints,
                        &descriptors);

        // Add the relevant image and feature data to the feature matcher. This allows
        // the feature matcher to control fine-grained things like multi-threading and
        // caching. For instance, the matcher may choose to write the descriptors to
        // disk and read them back as needed.
        std::lock_guard<std::mutex> lock(matcher_mutex_);
        matcher_->AddImage(image_filename, keypoints, descriptors, intrinsics);
    }

}  // namespace theia