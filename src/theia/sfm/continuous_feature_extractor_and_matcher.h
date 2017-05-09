//
// Created by maciek on 06.05.17.
//

#ifndef THEIA_CONTINUOUSFEATUREEXTRACTANDMATCH_H
#define THEIA_CONTINUOUSFEATUREEXTRACTANDMATCH_H


#include <string>
#include <thread>  // NOLINT
#include <vector>

#include "theia/image/descriptor/create_descriptor_extractor.h"
#include "theia/matching/create_feature_matcher.h"
#include "theia/matching/feature_matcher.h"
#include "theia/matching/feature_matcher_options.h"
#include "theia/sfm/exif_reader.h"

namespace theia {
    struct CameraIntrinsicsPrior;
    struct ImagePairMatch;

    class ContinuousFeatureExtractorAndMatcher {
    public:
        struct Options {
            // Number of threads for multithreading.
            int num_threads = 1;

            // If true, only images that contain EXIF focal length values have features
            // extracted and matched, and images that do not contain EXIF focal length
            // are not considered for the feature extraction and matching.
            bool only_calibrated_views = false;

            // The type of feature to use for feature extraction.
            DescriptorExtractorType descriptor_extractor_type =
                    DescriptorExtractorType::SIFT;

            // The density of features to extract. DENSE means more features are
            // extracted per image and SPARSE means fewer features per image are
            // extracted.
            FeatureDensity feature_density = FeatureDensity::NORMAL;

            // The features returned will be no larger than this size.
            int max_num_features = 16384;

            // Minimum number of inliers to consider the matches a good match.
            int min_num_inlier_matches = 30;

            // Matching strategy to use for establishing feature correspondences.
            MatchingStrategy matching_strategy;

            // Matching options for determining which feature matches are good matches.
            FeatureMatcherOptions feature_matcher_options;
        };

        explicit ContinuousFeatureExtractorAndMatcher(const Options &options);

        // Add an image to the image matcher queue.
        bool AddImage(const std::string &image_filepath);

        // Add an image with known camera intrinsics to the image matcher queue.
        bool AddImage(const std::string &image_filepath,
                      const CameraIntrinsicsPrior &intrinsics);

        // Assignes a mask to an image.
        // The mask is a black and white image, where black is 0.0 and white is 1.0.
        // The white part of the mask indicates the area for the keypoints extraction.
        bool AddMaskForFeaturesExtraction(const std::string &image_filepath,
                                          const std::string &mask_filepath);

        // Performs feature matching between all images provided by the image
        // filepaths. Features are extracted and matched between the images according
        // to the options passed in. Only matches that have passed geometric
        // verification are kept. EXIF data is parsed to determine the camera
        // intrinsics if available.
        void ExtractAndMatchFeatures(
                std::vector<CameraIntrinsicsPrior> *intrinsics,
                std::vector<ImagePairMatch> *matches);

    private:
        // Processes a single image by extracting EXIF information, extracting
        // features and descriptors, and adding the image to the matcher.
        void ProcessImage(const int i);

        const Options options_;

        // Local copies of the images to be matches, masks for use and any priors on
        // the camera intrinsics.
        std::vector<std::string> image_filepaths_;
        std::unordered_map<std::string, CameraIntrinsicsPrior> intrinsics_;
        std::unordered_map<std::string, std::string> image_masks_;

        // Exif reader for loading exif information. This object is created once so
        // that the EXIF focal length database does not have to be loaded multiple
        // times.
        ExifReader exif_reader_;

        // Feature matcher and mutex for thread-safe access.
        std::unique_ptr<FeatureMatcher> matcher_;
        std::mutex intrinsics_mutex_, matcher_mutex_;
    };

}  // namespace theia


#endif //THEIA_CONTINUOUSFEATUREEXTRACTANDMATCH_H
