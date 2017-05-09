//
// Created by maciek on 06.05.17.
//

#ifndef THEIA_CONTINUOUSRECONSTRUCTIONBUILDER_H
#define THEIA_CONTINUOUSRECONSTRUCTIONBUILDER_H


#include <memory>
#include <string>
#include <vector>

#include "theia/image/descriptor/create_descriptor_extractor.h"
#include "theia/matching/create_feature_matcher.h"
#include "theia/matching/feature_matcher_options.h"
#include "theia/sfm/reconstruction_estimator_options.h"
#include "theia/sfm/types.h"
#include "theia/util/util.h"

namespace theia {
    class ContinuousFeatureExtractorAndMatcher;

    class RandomNumberGenerator;

    class Reconstruction;

    class TrackBuilder;

    class ViewGraph;

    struct CameraIntrinsicsPrior;
    struct ImagePairMatch;

    struct ContinuousReconstructionBuilderOptions {
        // The random number generator used to generate random numbers through the
        // reconstruction building process. If this is a nullptr then the random
        // generator will be initialized based on the current time.
        std::shared_ptr<RandomNumberGenerator> rng;

        // Number of threads used. Each stage of the pipeline (feature extraction,
        // matching, estimation, etc.) will use this number of threads.
        int num_threads = 2;

        // By default, the ReconstructionBuilder will attempt to reconstruct as many
        // models as possible from the input data. If set to true, only the largest
        // connected component is reconstructed.
        bool reconstruct_largest_connected_component = false;

        // Set to true to only accept calibrated views (from EXIF or elsewhere) as
        // valid inputs to the reconstruction process. When uncalibrated views are
        // added to the reconstruction builder they are ignored with a LOG warning.
        bool only_calibrated_views = false;

        // Minimum allowable track length. Tracks that are too short are often not
        // well-constrained for triangulation and bundle adjustment.
        int min_track_length = 2;

        // Maximum allowable track length. Tracks that are too long are exceedingly
        // likely to contain outliers.
        int max_track_length = 50;

        // Minimum number of geometrically verified inliers that a view pair must have
        // in order to be considered a good match.
        int min_num_inlier_matches = 30;

        // Descriptor type for extracting features.
        // See //theia/image/descriptor/create_descriptor_extractor.h
        DescriptorExtractorType descriptor_type = DescriptorExtractorType::SIFT;

        // The density of features to extract. DENSE means more features are
        // extracted per image and SPARSE means fewer features per image are
        // extracted.
        FeatureDensity feature_density = FeatureDensity::NORMAL;

        // Matching strategy type.
        // See //theia/matching/create_feature_matcher.h
        MatchingStrategy matching_strategy = MatchingStrategy::BRUTE_FORCE;

        // Options for computing matches between images. Two view geometric
        // verification options are also part of these options.
        // See //theia/matching/feature_matcher_options.h
        FeatureMatcherOptions matching_options;

        // Options for estimating the reconstruction.
        // See //theia/sfm/reconstruction_estimator_options.h
        ReconstructionEstimatorOptions reconstruction_estimator_options;

        // If you want the matches to be saved, set this variable to the filename that
        // you want the matches to be written to. Image names, inlier matches, and
        // view metadata so that the view graph and tracks may be exactly
        // recreated.
        std::string output_matches_file;
    };

// Base class for building SfM reconstructions. This class will manage the
// entire reconstruction estimation process.
    class ContinuousReconstructionBuilder {
    public:
        explicit ContinuousReconstructionBuilder(const ContinuousReconstructionBuilderOptions &options);

        ~ContinuousReconstructionBuilder();

        // Add an image to the reconstruction.
        bool AddImage(const std::string &image_filepath);

        // Same as above, but with the camera intrinsics group specified to enable
        // shared camera intrinsics.
        bool AddImage(const std::string &image_filepath,
                      const CameraIntrinsicsGroupId camera_intrinsics_group);

        // Same as above, but with the camera priors manually specified.
        bool AddImageWithCameraIntrinsicsPrior(
                const std::string &image_filepath,
                const CameraIntrinsicsPrior &camera_intrinsics_prior);

        // Same as above, but with the camera intrinsics group specified to enable
        // shared camera intrinsics.
        bool AddImageWithCameraIntrinsicsPrior(
                const std::string &image_filepath,
                const CameraIntrinsicsPrior &camera_intrinsics_prior,
                const CameraIntrinsicsGroupId camera_intrinsics_group);

        // Add a match to the view graph. Either this method is repeatedly called or
        // ExtractAndMatchFeatures must be called.
        bool AddTwoViewMatch(const std::string &image1,
                             const std::string &image2,
                             const ImagePairMatch &matches);

        // Extracts features and performs matching with geometric verification.
        // Performs this action for all added files that are not yet computed.
        bool ExtractAndMatchFeatures();

        // Initializes the reconstruction and view graph explicitly. This method
        // should be used as an alternative to the Add* methods.
        //
        // NOTE: The ReconstructionBuilder takses ownership of the reconstruction and
        // view graph.
        void InitializeReconstructionAndViewGraph(Reconstruction *reconstruction,
                                                  ViewGraph *view_graph);

        // Estimates a Structure-from-Motion reconstruction using the specified
        // ReconstructionEstimator. Features are first extracted and matched if
        // necessary, then a reconstruction is estimated. Once a reconstruction has
        // been estimated, all views that have been successfully estimated are added
        // to the output vector and we estimate a reconstruction from the remaining
        // unestimated views. We repeat this process until no more views can be
        // successfully estimated.
        // Can be called after evry addition of the images.
        bool BuildReconstruction(std::vector<Reconstruction *> *reconstructions);

        bool BuildupReconstruction(Reconstruction *reconstruction,
                                   std::vector<std::string> *new_images_paths);

    private:
        // Adds the given matches as edges in the view graph.
        void AddMatchToViewGraph(const ViewId view_id1,
                                 const ViewId view_id2,
                                 const ImagePairMatch &image_matches);

        // Builds tracks from the two view inlier correspondences after geometric
        // verification.
        void AddTracksForMatch(const ViewId view_id1,
                               const ViewId view_id2,
                               const ImagePairMatch &image_matches);

        ContinuousReconstructionBuilderOptions options_;

        // SfM objects.
        std::unique_ptr<TrackBuilder> track_builder_;
        std::unique_ptr<Reconstruction> reconstruction_;
        std::unique_ptr<ViewGraph> view_graph_;

        // Container of image information.
        std::vector<std::string> image_filepaths_;

        // Module for performing feature extraction and matching.
        std::unique_ptr<ContinuousFeatureExtractorAndMatcher> feature_extractor_and_matcher_;

        DISALLOW_COPY_AND_ASSIGN(ContinuousReconstructionBuilder);
    };
}  // namespace theia

#endif //THEIA_CONTINUOUSRECONSTRUCTIONBUILDER_H
