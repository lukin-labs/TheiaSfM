//
// Created by maciek on 06.05.17.
//

#include "continuous_reconstruction_builder.h"

#include <glog/logging.h>
#include <memory>
#include <string>
#include <vector>

#include "theia/io/write_matches.h"
#include "theia/matching/image_pair_match.h"
#include "theia/sfm/camera_intrinsics_prior.h"
#include "theia/sfm/continuous_feature_extractor_and_matcher.h"
#include "theia/sfm/reconstruction.h"
#include "theia/sfm/reconstruction_estimator.h"
#include "theia/sfm/track_builder.h"
#include "theia/sfm/types.h"
#include "theia/sfm/view.h"
#include "theia/sfm/view_graph/view_graph.h"
#include "theia/util/filesystem.h"

namespace theia {
    namespace {

// Add the view to the reconstruction. If the camera intrinsics group id is set
// to an invalid group id then simply add the view to the reconstruction without
// shared camera intrinsics.
        bool AddViewToReconstruction(const std::string &image_filepath,
                                     const CameraIntrinsicsPrior *intrinsics,
                                     const CameraIntrinsicsGroupId intrinsics_group_id,
                                     Reconstruction *reconstruction) {
            std::string image_filename;
            CHECK(GetFilenameFromFilepath(image_filepath, true, &image_filename));

            // Add the image to the reconstruction.
            ViewId view_id;
            if (intrinsics_group_id == kInvalidCameraIntrinsicsGroupId) {
                view_id = reconstruction->AddView(image_filename);
            } else {
                view_id = reconstruction->AddView(image_filename, intrinsics_group_id);
            }

            if (view_id == kInvalidViewId) {
                LOG(INFO) << "Could not add " << image_filename
                          << " to the reconstruction.";
                return false;
            }

            // Add the camera intrinsics priors if available.
            if (intrinsics != nullptr) {
                View *view = reconstruction->MutableView(view_id);
                *view->MutableCameraIntrinsicsPrior() = *intrinsics;
            }
            return true;
        }

        Reconstruction *CreateEstimatedSubreconstruction(
                const Reconstruction &input_reconstruction) {
            std::unique_ptr<Reconstruction> subreconstruction(
                    new Reconstruction(input_reconstruction));
            const auto &view_ids = subreconstruction->ViewIds();
            for (const ViewId view_id : view_ids) {
                const View *view = subreconstruction->View(view_id);
                if (view == nullptr) {
                    continue;
                }

                if (!view->IsEstimated()) {
                    subreconstruction->RemoveView(view_id);
                }
            }

            const auto &track_ids = subreconstruction->TrackIds();
            for (const TrackId track_id : track_ids) {
                const Track *track = subreconstruction->Track(track_id);
                if (track == nullptr) {
                    continue;
                }

                if (!track->IsEstimated()) {
                    subreconstruction->RemoveTrack(track_id);
                }
            }
            return subreconstruction.release();
        }

        void RemoveEstimatedViewsAndTracks(Reconstruction *reconstruction,
                                           ViewGraph *view_graph) {
            const auto &view_ids = reconstruction->ViewIds();
            for (const ViewId view_id : view_ids) {
                const View *view = reconstruction->View(view_id);
                if (view == nullptr) {
                    continue;
                }

                if (view->IsEstimated()) {
                    reconstruction->RemoveView(view_id);
                    view_graph->RemoveView(view_id);
                }
            }

            const auto &track_ids = reconstruction->TrackIds();
            for (const TrackId track_id : track_ids) {
                const Track *track = reconstruction->Track(track_id);
                if (track == nullptr) {
                    continue;
                }

                if (track->IsEstimated()) {
                    reconstruction->RemoveTrack(track_id);
                }
            }
        }

    }  // namespace

    ContinuousReconstructionBuilder::ContinuousReconstructionBuilder(
            const ContinuousReconstructionBuilderOptions &options)
            : options_(options) {
        CHECK_GT(options.num_threads, 0);

        options_.reconstruction_estimator_options.rng = options.rng;

        reconstruction_.reset(new Reconstruction());
        view_graph_.reset(new ViewGraph());
        track_builder_.reset(
                new TrackBuilder(options.min_track_length, options.max_track_length));

        // Set up feature extraction and matching.
        ContinuousFeatureExtractorAndMatcher::Options feam_options;
        feam_options.num_threads = options_.num_threads;
        feam_options.only_calibrated_views = options_.only_calibrated_views;
        feam_options.num_threads = options_.num_threads;
        feam_options.descriptor_extractor_type =
                options_.descriptor_type;
        feam_options.feature_density = options_.feature_density;
        feam_options.min_num_inlier_matches = options_.min_num_inlier_matches;
        feam_options.matching_strategy = options_.matching_strategy;
        feam_options.feature_matcher_options = options_.matching_options;
        feam_options.feature_matcher_options.geometric_verification_options
                .min_num_inlier_matches = options_.min_num_inlier_matches;
        feam_options.feature_matcher_options.geometric_verification_options
                .estimate_twoview_info_options.rng = options_.rng;

        feature_extractor_and_matcher_.reset(
                new ContinuousFeatureExtractorAndMatcher(feam_options));
    }

    ContinuousReconstructionBuilder::~ContinuousReconstructionBuilder() {}

    bool ContinuousReconstructionBuilder::AddImage(const std::string &image_filepath) {
        return AddImage(image_filepath, kInvalidCameraIntrinsicsGroupId);
    }

    bool ContinuousReconstructionBuilder::AddImage(
            const std::string &image_filepath,
            const CameraIntrinsicsGroupId camera_intrinsics_group) {
        image_filepaths_.emplace_back(image_filepath);
        if (!AddViewToReconstruction(image_filepath,
                                     NULL,
                                     camera_intrinsics_group,
                                     reconstruction_.get())) {
            return false;
        }
        return feature_extractor_and_matcher_->AddImage(image_filepath);
    }

    bool ContinuousReconstructionBuilder::AddImageWithCameraIntrinsicsPrior(
            const std::string &image_filepath,
            const CameraIntrinsicsPrior &camera_intrinsics_prior) {
        return AddImageWithCameraIntrinsicsPrior(
                image_filepath, camera_intrinsics_prior, kInvalidCameraIntrinsicsGroupId);
    }

    bool ContinuousReconstructionBuilder::AddImageWithCameraIntrinsicsPrior(
            const std::string &image_filepath,
            const CameraIntrinsicsPrior &camera_intrinsics_prior,
            const CameraIntrinsicsGroupId camera_intrinsics_group) {
        image_filepaths_.emplace_back(image_filepath);
        if (!AddViewToReconstruction(image_filepath,
                                     &camera_intrinsics_prior,
                                     camera_intrinsics_group,
                                     reconstruction_.get())) {
            return false;
        }
        return feature_extractor_and_matcher_->AddImage(image_filepath,
                                                        camera_intrinsics_prior);
    }

    bool ContinuousReconstructionBuilder::ExtractAndMatchFeatures() {
        CHECK_EQ(view_graph_->NumViews(), 0) << "Cannot call ExtractAndMatchFeatures "
                    "after TwoViewMatches has been "
                    "called.";

        // Extract features and obtain the feature matches.
        std::vector<ImagePairMatch> matches;
        std::vector<CameraIntrinsicsPrior> camera_intrinsics_priors;
        feature_extractor_and_matcher_->ExtractAndMatchFeatures(
                &camera_intrinsics_priors, &matches);

        // Add the EXIF data to each view.
        std::vector<std::string> image_filenames(image_filepaths_.size());
        for (int i = 0; i < image_filepaths_.size(); i++) {
            CHECK(GetFilenameFromFilepath(image_filepaths_[i], true,
                                          &image_filenames[i]));

            // Add the camera intrinsic prior information to the view.
            const ViewId view_id = reconstruction_->ViewIdFromName(image_filenames[i]);
            View *view = reconstruction_->MutableView(view_id);
            *view->MutableCameraIntrinsicsPrior() = camera_intrinsics_priors[i];
        }

        // Write the matches to a file if it exists.
        if (options_.output_matches_file.length() > 0) {
            LOG(INFO) << "Writing matches to file: " << options_.output_matches_file;
            CHECK(WriteMatchesAndGeometry(options_.output_matches_file,
                                          image_filenames,
                                          camera_intrinsics_priors,
                                          matches))
            << "Could not write the matches to " << options_.output_matches_file;
        }

        // Add the matches to the view graph and reconstruction.
        for (const auto &match : matches) {
            AddTwoViewMatch(match.image1, match.image2, match);
        }

        return true;
    }

    bool ContinuousReconstructionBuilder::AddTwoViewMatch(const std::string &image1,
                                                          const std::string &image2,
                                                          const ImagePairMatch &matches) {
        // Get view ids from names and check that the views are valid (i.e. that
        // they have been added to the reconstruction).
        const ViewId view_id1 = reconstruction_->ViewIdFromName(image1);
        const ViewId view_id2 = reconstruction_->ViewIdFromName(image2);
        CHECK_NE(view_id1, kInvalidViewId)
            << "Tried to add a view with the name " << image1
            << " to the view graph but does not exist in the reconstruction.";
        CHECK_NE(view_id2, kInvalidViewId)
            << "Tried to add a view with the name " << image2
            << " to the view graph but does not exist in the reconstruction.";

        // If we only want calibrated views, do not add the match if it contains an
        // uncalibrated view since it will add uncalibrated views to the tracks.
        const View *view1 = reconstruction_->View(view_id1);
        const View *view2 = reconstruction_->View(view_id2);
        if (options_.only_calibrated_views &&
            (!view1->CameraIntrinsicsPrior().focal_length.is_set ||
             !view2->CameraIntrinsicsPrior().focal_length.is_set)) {
            return true;
        }

        // Add valid matches to view graph.
        AddMatchToViewGraph(view_id1, view_id2, matches);

        // Add tracks to the track builder.
        AddTracksForMatch(view_id1, view_id2, matches);

        return true;
    }

    void ContinuousReconstructionBuilder::InitializeReconstructionAndViewGraph(
            Reconstruction *reconstruction, ViewGraph *view_graph) {
        reconstruction_.reset(std::move(reconstruction));
        view_graph_.reset(std::move(view_graph));
    }

    bool ContinuousReconstructionBuilder::BuildReconstruction(
            std::vector<Reconstruction *> *reconstructions) {
        CHECK_GE(view_graph_->NumViews(), 2) << "At least 2 images must be provided "
                    "in order to create a "
                    "reconstruction.";

        // Build tracks if they were not explicitly specified.
        if (reconstruction_->NumTracks() == 0) {
            track_builder_->BuildTracks(reconstruction_.get());
        }

        while (reconstruction_->NumViews() > 1) {
            LOG(INFO) << "Attempting to reconstruct " << reconstruction_->NumViews()
                      << " images from " << view_graph_->NumEdges()
                      << " two view matches.";

            std::unique_ptr<ReconstructionEstimator> reconstruction_estimator(
                    ReconstructionEstimator::Create(
                            options_.reconstruction_estimator_options));

            const auto &summary = reconstruction_estimator->Estimate(
                    view_graph_.get(), reconstruction_.get());

            // If a reconstruction can no longer be estimated, return.
            if (!summary.success) {
                return reconstructions->size() > 0;
            }

            LOG(INFO)
                    << "\nReconstruction estimation statistics: "
                    << "\n\tNum estimated views = " << summary.estimated_views.size()
                    << "\n\tNum input views = " << reconstruction_->NumViews()
                    << "\n\tNum estimated tracks = " << summary.estimated_tracks.size()
                    << "\n\tNum input tracks = " << reconstruction_->NumTracks()
                    << "\n\tPose estimation time = " << summary.pose_estimation_time
                    << "\n\tTriangulation time = " << summary.triangulation_time
                    << "\n\tBundle Adjustment time = " << summary.bundle_adjustment_time
                    << "\n\tTotal time = " << summary.total_time
                    << "\n\n" << summary.message;

            // Remove estimated views and tracks and attempt to create a reconstruction
            // from the remaining unestimated parts.
            reconstructions->emplace_back(
                    CreateEstimatedSubreconstruction(*reconstruction_));
            RemoveEstimatedViewsAndTracks(reconstruction_.get(), view_graph_.get());

            // Exit after the first reconstruction estimation if only the single largest
            // reconstruction is desired.
            if (options_.reconstruct_largest_connected_component) {
                return reconstructions->size() > 0;
            }

            if (reconstruction_->NumViews() < 3) {
                LOG(INFO) << "No more reconstructions can be estimated.";
                return reconstructions->size() > 0;
            }
        }
        return true;
    }

    void ContinuousReconstructionBuilder::AddMatchToViewGraph(
            const ViewId view_id1,
            const ViewId view_id2,
            const ImagePairMatch &image_matches) {
        // Add the view pair to the reconstruction. The view graph requires the two
        // view info
        // to specify the transformation from the smaller view id to the larger view
        // id. We swap the cameras here if that is not already the case.
        TwoViewInfo twoview_info = image_matches.twoview_info;
        if (view_id1 > view_id2) {
            SwapCameras(&twoview_info);
        }

        view_graph_->AddEdge(view_id1, view_id2, twoview_info);
    }

    void ContinuousReconstructionBuilder::AddTracksForMatch(const ViewId view_id1,
                                                            const ViewId view_id2,
                                                            const ImagePairMatch &matches) {
        for (const auto &match : matches.correspondences) {
            track_builder_->AddFeatureCorrespondence(view_id1, match.feature1,
                                                     view_id2, match.feature2);
        }
    }

}  // namespace theia