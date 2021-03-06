//
// Created by maciek on 06.05.17.
//

#ifndef THEIA_BUILD_CONTINUOUS_RECONSTRUCTION_PARAMS_H
#define THEIA_BUILD_CONTINUOUS_RECONSTRUCTION_PARAMS_H

#include "applications/command_line_helpers.h"

// Input/output files.
DEFINE_string(images, "/home/maciek/Datasets/mobile_floor_2fps/*.png", "Wildcard of images to reconstruct.");
DEFINE_string(image_masks, "", "Wildcard of image masks to reconstruct.");
DEFINE_string(matches_file, "", "Filename of the matches file.");
DEFINE_string(calibration_file, "",
              "Calibration file containing image calibration data.");
DEFINE_string(
        output_matches_file, "",
        "File to write the two-view matches to. This file can be used in "
                "future iterations as input to the reconstruction builder. Leave empty if "
                "you do not want to output matches.");
DEFINE_string(
        output_reconstruction, "/home/maciek/Datasets/mobile_floor_2fps/results/rec",
        "Filename to write reconstruction to. The filename will be appended with "
                "the reconstruction number if multiple reconstructions are created.");

// Multithreading.
DEFINE_int32(num_threads, 4,
             "Number of threads to use for feature extraction and matching.");

// Feature and matching options.
DEFINE_string(descriptor, "SIFT",
              "Type of feature descriptor to use. Must be one of the following: "
                      "SIFT");
DEFINE_string(feature_density, "DENSE",
              "Set to SPARSE, NORMAL, or DENSE to extract fewer or more "
                      "features from each image.");
DEFINE_string(matching_strategy, "CASCADE_HASHING",
              "Strategy used to match features. Must be BRUTE_FORCE "
                      " or CASCADE_HASHING");
DEFINE_bool(match_out_of_core, false,
            "Perform matching out of core by saving features to disk and "
                    "reading them as needed. Set to false to perform matching all in "
                    "memory.");
DEFINE_string(matching_working_directory, "/home/maciek/Datasets/mobile_floor_2fps/matches/",
              "Directory used during matching to store features for "
                      "out-of-core matching.");
DEFINE_int32(matching_max_num_images_in_cache, 128,
             "Maximum number of images to store in the LRU cache during "
                     "feature matching. The higher this number is the more memory is "
                     "consumed during matching.");
DEFINE_double(lowes_ratio, 0.8, "Lowes ratio used for feature matching.");
DEFINE_double(max_sampson_error_for_verified_match, 4.0,
              "Maximum sampson error for a match to be considered "
                      "geometrically valid. This threshold is relative to an image "
                      "with a width of 1024 pixels and will be appropriately scaled "
                      "for images with different resolutions.");
DEFINE_int32(min_num_inliers_for_valid_match, 30,
             "Minimum number of geometrically verified inliers that a pair on "
                     "images must have in order to be considered a valid two-view "
                     "match.");
DEFINE_bool(bundle_adjust_two_view_geometry, true,
            "Set to false to turn off 2-view BA.");
DEFINE_bool(keep_only_symmetric_matches, true,
            "Performs two-way matching and keeps symmetric matches.");

// Reconstruction building options.
DEFINE_string(reconstruction_estimator, "INCREMENTAL",
              "Type of SfM reconstruction estimation to use.");
DEFINE_bool(reconstruct_largest_connected_component, false,
            "If set to true, only the single largest connected component is "
                    "reconstructed. Otherwise, as many models as possible are "
                    "estimated.");
DEFINE_bool(shared_calibration, false,
            "Set to true if all camera intrinsic parameters should be shared "
                    "as a single set of intrinsics. This is useful, for instance, if "
                    "all images in the reconstruction were taken with the same "
                    "camera.");
DEFINE_bool(only_calibrated_views, false,
            "Set to true to only reconstruct the views where calibration is "
                    "provided or can be extracted from EXIF");
DEFINE_int32(min_track_length, 2, "Minimum length of a track.");
DEFINE_int32(max_track_length, 50, "Maximum length of a track.");
DEFINE_string(intrinsics_to_optimize,
              "NONE",
              "Set to control which intrinsics parameters are optimized during "
                      "bundle adjustment.");
DEFINE_double(max_reprojection_error_pixels, 4.0,
              "Maximum reprojection error for a correspondence to be "
                      "considered an inlier after bundle adjustment.");

// Global SfM options.
DEFINE_string(global_rotation_estimator, "ROBUST_L1L2",
              "Type of global rotation estimation to use for global SfM.");
DEFINE_string(global_position_estimator, "NONLINEAR",
              "Type of global position estimation to use for global SfM.");
DEFINE_bool(refine_relative_translations_after_rotation_estimation, true,
            "Refine the relative translation estimation after computing the "
                    "absolute rotations. This can help improve the accuracy of the "
                    "position estimation.");
DEFINE_double(post_rotation_filtering_degrees, 5.0,
              "Max degrees difference in relative rotation and rotation "
                      "estimates for rotation filtering.");
DEFINE_bool(extract_maximal_rigid_subgraph, false,
            "If true, only cameras that are well-conditioned for position "
                    "estimation will be used for global position estimation.");
DEFINE_bool(filter_relative_translations_with_1dsfm, true,
            "Filter relative translation estimations with the 1DSfM algorithm "
                    "to potentially remove outlier relativep oses for position "
                    "estimation.");
DEFINE_bool(refine_camera_positions_and_points_after_position_estimation, true,
            "After estimating positions in Global SfM we can refine only "
                    "camera positions and 3D point locations, holding camera "
                    "intrinsics and rotations constant. This often improves the "
                    "stability of bundle adjustment when the camera intrinsics are "
                    "inaccurate.");
DEFINE_int32(num_retriangulation_iterations, 1,
             "Number of times to retriangulate any unestimated tracks. Bundle "
                     "adjustment is performed after retriangulation.");

// Nonlinear position estimation options.
DEFINE_int32(
        position_estimation_min_num_tracks_per_view, 0,
        "Minimum number of point to camera constraints for position estimation.");
DEFINE_double(position_estimation_robust_loss_width, 0.1,
              "Robust loss width to use for position estimation.");

// Incremental SfM options.
DEFINE_double(absolute_pose_reprojection_error_threshold, 4.0,
              "The inlier threshold for absolute pose estimation. This "
                      "threshold is relative to an image with a width of 1024 pixels "
                      "and will be appropriately scaled based on the input image "
                      "resolutions.");
DEFINE_int32(min_num_absolute_pose_inliers, 30,
             "Minimum number of inliers in order for absolute pose estimation "
                     "to be considered successful.");
DEFINE_double(full_bundle_adjustment_growth_percent, 5.0,
              "Full BA is only triggered for incremental SfM when the "
                      "reconstruction has growth by this percent since the last time "
                      "full BA was used.");
DEFINE_int32(partial_bundle_adjustment_num_views, 20,
             "When full BA is not being run, partial BA is executed on a "
                     "constant number of views specified by this parameter.");

// Triangulation options.
DEFINE_double(min_triangulation_angle_degrees, 4.0,
              "Minimum angle between views for triangulation.");
DEFINE_double(
        triangulation_reprojection_error_pixels, 15.0,
        "Max allowable reprojection error on initial triangulation of points.");
DEFINE_bool(bundle_adjust_tracks, true,
            "Set to true to optimize tracks immediately upon estimation.");

// Bundle adjustment parameters.
DEFINE_string(bundle_adjustment_robust_loss_function, "NONE",
              "By setting this to an option other than NONE, a robust loss "
                      "function will be used during bundle adjustment which can "
                      "improve robustness to outliers. Options are NONE, HUBER, "
                      "SOFTLONE, CAUCHY, ARCTAN, and TUKEY.");
DEFINE_double(bundle_adjustment_robust_loss_width, 10.0,
              "If the BA loss function is not NONE, then this value controls "
                      "where the robust loss begins with respect to reprojection error "
                      "in pixels.");

// Track Subsampling parameters.
DEFINE_bool(subsample_tracks_for_bundle_adjustment, false,
            "Set to true to subsample tracks used for bundle adjustment. This "
                    "can help improve efficiency of bundle adjustment dramatically "
                    "when used properly.");
DEFINE_int32(track_subset_selection_long_track_length_threshold, 10,
             "When track subsampling is enabled, longer tracks are chosen with "
                     "a higher probability with the track length capped to this value "
                     "for selection.");
DEFINE_int32(track_selection_image_grid_cell_size_pixels, 100,
             "When track subsampling is enabled, tracks are chosen such that "
                     "each view has a good spatial coverage. This is achieved by "
                     "binning tracks into an image grid in each view and choosing the "
                     "best tracks in each grid cell to guarantee spatial coverage. The "
                     "image grid cells are defined to be this width in pixels.");
DEFINE_int32(min_num_optimized_tracks_per_view, 100,
             "When track subsampling is enabled, tracks are selected such that "
                     "each view observes a minimum number of optimized tracks.");

using theia::Reconstruction;
using theia::ContinuousReconstructionBuilder;
using theia::ContinuousReconstructionBuilderOptions;

// Sets the feature extraction, matching, and reconstruction options based on
// the command line flags. There are many more options beside just these located
// in //theia/vision/sfm/reconstruction_builder.h
ContinuousReconstructionBuilderOptions SetReconstructionBuilderOptions() {
    ContinuousReconstructionBuilderOptions options;
    options.num_threads = FLAGS_num_threads;
    options.output_matches_file = FLAGS_output_matches_file;

    options.descriptor_type = StringToDescriptorExtractorType(FLAGS_descriptor);
    options.feature_density = StringToFeatureDensity(FLAGS_feature_density);
    options.matching_options.match_out_of_core = FLAGS_match_out_of_core;
    options.matching_options.keypoints_and_descriptors_output_dir =
            FLAGS_matching_working_directory;
    options.matching_options.cache_capacity =
            FLAGS_matching_max_num_images_in_cache;
    options.matching_strategy = StringToMatchingStrategyType(FLAGS_matching_strategy);
    options.matching_options.lowes_ratio = (float) FLAGS_lowes_ratio;
    options.matching_options.keep_only_symmetric_matches = FLAGS_keep_only_symmetric_matches;
    options.min_num_inlier_matches = FLAGS_min_num_inliers_for_valid_match;
    options.matching_options.perform_geometric_verification = true;
    options.matching_options.geometric_verification_options.estimate_twoview_info_options.max_sampson_error_pixels =
            FLAGS_max_sampson_error_for_verified_match;
    options.matching_options.geometric_verification_options.bundle_adjustment =
            FLAGS_bundle_adjust_two_view_geometry;
    options.matching_options.geometric_verification_options.triangulation_max_reprojection_error =
            FLAGS_triangulation_reprojection_error_pixels;
    options.matching_options.geometric_verification_options.min_triangulation_angle_degrees =
            FLAGS_min_triangulation_angle_degrees;
    options.matching_options.geometric_verification_options.final_max_reprojection_error =
            FLAGS_max_reprojection_error_pixels;

    options.min_track_length = FLAGS_min_track_length;
    options.max_track_length = FLAGS_max_track_length;

    // Reconstruction Estimator Options.
    theia::ReconstructionEstimatorOptions &reconstruction_estimator_options =
            options.reconstruction_estimator_options;
    reconstruction_estimator_options.min_num_two_view_inliers = FLAGS_min_num_inliers_for_valid_match;
    reconstruction_estimator_options.num_threads = FLAGS_num_threads;
    reconstruction_estimator_options.intrinsics_to_optimize =
            StringToOptimizeIntrinsicsType(FLAGS_intrinsics_to_optimize);
    options.reconstruct_largest_connected_component = FLAGS_reconstruct_largest_connected_component;
    options.only_calibrated_views = FLAGS_only_calibrated_views;
    reconstruction_estimator_options.max_reprojection_error_in_pixels = FLAGS_max_reprojection_error_pixels;

    // Which type of SfM pipeline to use (e.g., incremental, global, etc.);
    reconstruction_estimator_options.reconstruction_estimator_type =
            StringToReconstructionEstimatorType(FLAGS_reconstruction_estimator);

    // Global SfM Options.
    reconstruction_estimator_options.global_rotation_estimator_type =
            StringToRotationEstimatorType(FLAGS_global_rotation_estimator);
    reconstruction_estimator_options.global_position_estimator_type =
            StringToPositionEstimatorType(FLAGS_global_position_estimator);
    reconstruction_estimator_options.num_retriangulation_iterations =
            FLAGS_num_retriangulation_iterations;
    reconstruction_estimator_options
            .refine_relative_translations_after_rotation_estimation =
            FLAGS_refine_relative_translations_after_rotation_estimation;
    reconstruction_estimator_options.extract_maximal_rigid_subgraph =
            FLAGS_extract_maximal_rigid_subgraph;
    reconstruction_estimator_options.filter_relative_translations_with_1dsfm =
            FLAGS_filter_relative_translations_with_1dsfm;
    reconstruction_estimator_options
            .rotation_filtering_max_difference_degrees =
            FLAGS_post_rotation_filtering_degrees;
    reconstruction_estimator_options.nonlinear_position_estimator_options
            .min_num_points_per_view =
            FLAGS_position_estimation_min_num_tracks_per_view;
    reconstruction_estimator_options
            .refine_camera_positions_and_points_after_position_estimation =
            FLAGS_refine_camera_positions_and_points_after_position_estimation;

    // Incremental SfM Options.
    reconstruction_estimator_options
            .absolute_pose_reprojection_error_threshold =
            FLAGS_absolute_pose_reprojection_error_threshold;
    reconstruction_estimator_options.min_num_absolute_pose_inliers =
            FLAGS_min_num_absolute_pose_inliers;
    reconstruction_estimator_options
            .full_bundle_adjustment_growth_percent =
            FLAGS_full_bundle_adjustment_growth_percent;
    reconstruction_estimator_options.partial_bundle_adjustment_num_views =
            FLAGS_partial_bundle_adjustment_num_views;

    // Triangulation options (used by all SfM pipelines).
    reconstruction_estimator_options.min_triangulation_angle_degrees =
            FLAGS_min_triangulation_angle_degrees;
    reconstruction_estimator_options
            .triangulation_max_reprojection_error_in_pixels =
            FLAGS_triangulation_reprojection_error_pixels;
    reconstruction_estimator_options.bundle_adjust_tracks =
            FLAGS_bundle_adjust_tracks;

    // Bundle adjustment options (used by all SfM pipelines).
    reconstruction_estimator_options.bundle_adjustment_loss_function_type =
            StringToLossFunction(FLAGS_bundle_adjustment_robust_loss_function);
    reconstruction_estimator_options.bundle_adjustment_robust_loss_width =
            FLAGS_bundle_adjustment_robust_loss_width;

    // Track subsampling options.
    reconstruction_estimator_options.subsample_tracks_for_bundle_adjustment =
            FLAGS_subsample_tracks_for_bundle_adjustment;
    reconstruction_estimator_options
            .track_subset_selection_long_track_length_threshold =
            FLAGS_track_subset_selection_long_track_length_threshold;
    reconstruction_estimator_options.track_selection_image_grid_cell_size_pixels =
            FLAGS_track_selection_image_grid_cell_size_pixels;
    reconstruction_estimator_options.min_num_optimized_tracks_per_view =
            FLAGS_min_num_optimized_tracks_per_view;
    return options;
}

#endif //THEIA_BUILD_CONTINUOUS_RECONSTRUCTION_PARAMS_H
