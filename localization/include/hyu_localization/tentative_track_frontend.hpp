// Copyright 2026 shchon11
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef HYU_LOCALIZATION__TENTATIVE_TRACK_FRONTEND_HPP_
#define HYU_LOCALIZATION__TENTATIVE_TRACK_FRONTEND_HPP_

#include <Eigen/Core>
#include <Eigen/LU>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <utility>
#include <vector>

namespace hyu_localization
{

// Delayed data association frontend (tentative tracks).
//
// The legacy pipeline confirmed every observation into the graph the frame
// it arrived: nearest-neighbour + chi2, then an irreversible vertex/edge.
// Every founding heuristic layered on top of it (creation range cap, split
// near/far rule, crowd radius, soft-founding covariance) existed to stop a
// SINGLE bad frame from polluting the map, because pollution could never be
// undone. This frontend removes the root cause instead: an observation that
// matches no confirmed landmark founds a TENTATIVE track that stays outside
// the graph. Only a track that survives repeated re-observation and whose
// fused position has converged is promoted to a landmark — and at promotion
// its full observation history is replayed into the graph against the
// original keyframes, so the delay costs no information.
//
// The seam failure (offset twins) dies structurally: a lap-return
// observation offset ~1-2 m by drift founds a track as before, but the
// track is invisible to the optimizer, and promotion is HELD while a
// confirmed landmark sits within hold radius — the drift is for the loop
// closure machinery (gate anchor, robust backend) to resolve, not for a
// duplicate landmark to absorb.
//
// Association is a frame-global greedy over ascending chi2 with mutual
// exclusion across the union of {confirmed landmarks, tentative tracks},
// replacing the observation-order-dependent claimed[] greedy.
//
// Header-only and ROS/g2o-free (mirrors gate_anchor.hpp) so the association
// logic is unit-testable headless. All positions/covariances are map frame
// unless suffixed otherwise.

inline constexpr std::size_t kFrontendColorCount = 5U;

struct FrontendParams
{
  // Confirmed-landmark association (moved verbatim from the legacy node so
  // behaviour for the matched path is unchanged).
  double association_max_distance_m{1.2};
  double association_gate_chi2{5.991};
  double association_ambiguity_ratio{0.85};
  double association_inflation_per_meter{0.01};
  double association_max_inflation{4.0};
  // Ceiling on the drift-widened duplicate-pair separation (see the
  // ambiguity check): must stay below the minimum real same-color cone
  // spacing (~3 m), or the widening would reclassify genuinely ambiguous
  // neighbour pairs as duplicates.
  double duplicate_pair_max_separation_m{2.5};

  // Tentative-track association: tracks are fresh (no drift inflation), so
  // the gate is a plain chi2 on track-plus-observation covariance with a
  // Euclidean pre-gate to bound the search.
  double track_gate_chi2{9.21};
  double track_gate_max_distance_m{2.4};

  // Heading-leverage softness for the PROMOTION DECISION only: unoptimized
  // heading error displaces an observation by ~sigma_psi * range, so track
  // fusion inflates each observation by that much. The replayed graph edges
  // keep the raw sensor covariance — the optimizer estimates pose and
  // landmark jointly, which is the principled home for heading error.
  double heading_lever_sigma_rad{0.035};

  // Promotion: at least this many hits AND the fused position's worst-axis
  // sigma below the cap. A far-founded track (large lever sigma at birth)
  // therefore needs near passes before it can promote.
  int promote_min_hits{3};
  double promote_max_position_sigma_m{0.35};
  // Far-promotion relief: a cone only ever seen at range (outside of a
  // corner, occluded near pass) carries a lever-sigma floor above the cap
  // and would NEVER promote — a permanent hole in the map. With enough
  // corroborating hits it promotes anyway, carrying its honest (wide)
  // covariance into the graph; the optimizer refines it like any landmark.
  int promote_far_min_hits{6};
  double promote_far_max_sigma_m{0.7};
  // Promotion is HELD (not killed) while a STALE confirmed landmark sits
  // within this radius: a drift-offset re-sighting of an existing cone whose
  // own association broke (its observations now feed the track, so it
  // stopped being seen). A FRESH neighbour inside the radius is the
  // opposite case — cone and track observed simultaneously means two real
  // cones (a physical cone's returns cannot land in two places), so the
  // track must promote. Holding on radius alone ate every real cone spaced
  // under 2 m (autocross serpentine, 2026-07-18: 47/48 missing cones had a
  // neighbour within 2.0 m).
  double promote_hold_radius_m{2.0};
  double promote_hold_stale_travel_m{5.0};
  // The drift-scaled hold radius (base + 3*sqrt(gate inflation)) applies
  // only to neighbours at least this stale. Seam-crossing twins have a
  // nearly lap-old original (hundreds of meters); a serpentine's previous
  // leg is merely FOV-stale (15-30 m), and inflating its hold would eat the
  // real cones of the next leg.
  double promote_hold_drift_stale_m{60.0};

  // Track death: DISTANCE driven while the track was expected visible but
  // not hit (cheap to be aggressive — a killed track re-founds on the next
  // sighting and never touched the map), plus an age cap so unpromotable
  // tracks cannot linger as silent observation absorbers. Metres, not frame
  // or keyframe counts: a count budget silently rescales whenever the
  // processing cadence changes (2026-08-24: halving keyframe_distance
  // halved every count budget and ate real cones). A stationary car burns
  // no blind travel — flicker at standstill cannot kill a track.
  double kill_blind_travel_m{1.5};
  double kill_unpromoted_travel_m{25.0};

  // Replay buffer cap per track (hits keep counting past it).
  std::size_t max_pending_observations{12};
};

struct FrontendObservation
{
  Eigen::Vector2d map_point{Eigen::Vector2d::Zero()};
  Eigen::Matrix2d map_covariance{Eigen::Matrix2d::Identity()};
  // Measurement re-expressed in the owning keyframe's frame, kept verbatim
  // for graph replay at promotion.
  Eigen::Vector2d keyframe_measurement{Eigen::Vector2d::Zero()};
  Eigen::Matrix2d keyframe_covariance{Eigen::Matrix2d::Identity()};
  int pose_graph_id{-1};
  // Sensor range of the raw measurement (heading-lever arm).
  double range_m{0.0};
  // Colour id: indexes color votes; kFrontendColorCount - 1 = unknown.
  std::uint8_t color{static_cast<std::uint8_t>(kFrontendColorCount - 1U)};
};

struct FrontendConfirmedLandmark
{
  Eigen::Vector2d position{Eigen::Vector2d::Zero()};
  Eigen::Matrix2d covariance{Eigen::Matrix2d::Identity()};
  double last_seen_traveled_m{0.0};
};

struct FrontendPendingObservation
{
  int pose_graph_id{-1};
  Eigen::Vector2d keyframe_measurement{Eigen::Vector2d::Zero()};
  Eigen::Matrix2d keyframe_covariance{Eigen::Matrix2d::Identity()};
};

struct FrontendPromotion
{
  Eigen::Vector2d position{Eigen::Vector2d::Zero()};
  Eigen::Matrix2d covariance{Eigen::Matrix2d::Identity()};
  // Total hits of the promoting track — consumers with a stricter admission
  // bar (localization-mode map repair) gate on it.
  int hits{0};
  std::uint8_t color{static_cast<std::uint8_t>(kFrontendColorCount - 1U)};
  std::array<std::uint16_t, kFrontendColorCount> color_votes{};
  std::vector<FrontendPendingObservation> observations;
};

struct FrontendConfirmedMatch
{
  std::size_t observation_index{0U};
  std::size_t landmark_index{0U};
};

// Read-only view of a converged (promotable) track, for consumers that want
// to SEE the cone before the graph admits it (map publication): position,
// fused covariance and the current majority colour.
struct FrontendTrackPreview
{
  Eigen::Vector2d position{Eigen::Vector2d::Zero()};
  Eigen::Matrix2d covariance{Eigen::Matrix2d::Identity()};
  std::uint8_t color{static_cast<std::uint8_t>(kFrontendColorCount - 1U)};
};

struct FrontendFrameResult
{
  std::vector<FrontendConfirmedMatch> confirmed_matches;
  std::vector<std::size_t> ambiguous_observations;
  std::vector<FrontendPromotion> promotions;
  std::size_t new_tracks{0U};
  std::size_t updated_tracks{0U};
  std::size_t killed_tracks{0U};
  // Promotable tracks kept back this frame (any hold); caller_held_promotions
  // is the subset held by the caller's promotion_hold predicate.
  std::size_t held_promotions{0U};
  std::size_t caller_held_promotions{0U};
};

struct TentativeTrack
{
  Eigen::Vector2d position{Eigen::Vector2d::Zero()};
  Eigen::Matrix2d covariance{Eigen::Matrix2d::Identity()};
  std::array<std::uint16_t, kFrontendColorCount> color_votes{};
  int hits{0};
  // Metres driven while this track was expected visible but not hit,
  // since its last hit (see FrontendParams::kill_blind_travel_m).
  double blind_travel_m{0.0};
  double first_seen_traveled_m{0.0};
  double last_seen_traveled_m{0.0};
  // Smallest heading-lever sigma over all hits. The lever error is
  // correlated across frames (one heading bias displaces every sighting the
  // same way), so per-frame fusion must not average it away; the best any
  // observation history can promise is the sigma of its CLOSEST sighting.
  double min_lever_sigma_m{std::numeric_limits<double>::max()};
  std::vector<FrontendPendingObservation> pending;
};

namespace frontend_detail
{

inline double maxAxisSigma(const Eigen::Matrix2d & covariance)
{
  const double half_trace = 0.5 * (covariance(0, 0) + covariance(1, 1));
  const double det = covariance.determinant();
  const double discriminant = std::max(0.0, half_trace * half_trace - det);
  return std::sqrt(std::max(0.0, half_trace + std::sqrt(discriminant)));
}

inline bool invertibleInnovation(const Eigen::Matrix2d & covariance)
{
  return covariance.allFinite() && covariance.determinant() > 0.0;
}

}  // namespace frontend_detail

class TentativeTrackFrontend
{
public:
  explicit TentativeTrackFrontend(const FrontendParams & params)
  : params_(params) {}

  const std::vector<TentativeTrack> & tracks() const {return tracks_;}

  void reset()
  {
    tracks_.clear();
    last_processed_traveled_m_ = -1.0;
  }

  // Read-only previews of every track that already satisfies the promotion
  // criteria (converged real cones -- typically the ones a hold is keeping
  // out of the graph). Lets the map publication show them to the planner
  // while the seam is pending: with these cones missing the local planner
  // went blind over the last stretch of lap 1 (2026-08-24 user-observed
  // wobble + crash). Graph admission is unaffected.
  std::vector<FrontendTrackPreview> promotablePreviews() const
  {
    std::vector<FrontendTrackPreview> previews;
    for (const TentativeTrack & track : tracks_) {
      if (isPromotable(track)) {
        previews.push_back(
          FrontendTrackPreview{
            track.position, track.covariance, majorityColor(track.color_votes)});
      }
    }
    return previews;
  }

  // Pull every track that already satisfies the promotion criteria (hits and
  // converged sigma) out of the frontend as promotions, IGNORING both holds
  // (the stale-landmark drift-twin hold and the caller's promotion_hold).
  // For the moments the caller is about to reset() the frontend -- a large
  // pose correction, or the map freeze -- so that converged real cones that
  // were merely waiting are handed over with their full replay history
  // instead of dying with the tracks. The caller's twin suppression is the
  // remaining guard against drift twins. Non-promotable tracks stay.
  std::vector<FrontendPromotion> drainPromotable()
  {
    std::vector<FrontendPromotion> promotions;
    std::vector<TentativeTrack> survivors;
    survivors.reserve(tracks_.size());
    for (TentativeTrack & track : tracks_) {
      if (isPromotable(track)) {
        promotions.push_back(makePromotion(track));
      } else {
        survivors.push_back(std::move(track));
      }
    }
    tracks_ = std::move(survivors);
    return promotions;
  }

  // One perception frame. expected_visible reports whether a map point
  // should currently be observable (range/FOV from the live pose); tracks
  // outside it neither miss nor die, so occlusion and FOV exits do not eat
  // the miss budget (the 2026-07-17 reaper lesson).
  //
  // promotion_hold (optional) lets the caller HOLD a promotable track by
  // position: the track survives and keeps accumulating hits, colour votes
  // and replay history, and is promoted on the first frame the predicate
  // releases it. This is how registration near the start-area map waits
  // for the lap seam without the track being torn down and re-founded
  // (the old node-side deferral removed the track here, so every later
  // sighting founded a fresh copy; at the flush the EARLIEST copy -- far,
  // colourless -- won and the later colour votes were discarded as twins).
  // A track held this way is exempt from kill_unpromoted_travel_m: it is a
  // converged real cone that is waiting on the caller, not a stray.
  FrontendFrameResult processFrame(
    const std::vector<FrontendObservation> & observations,
    const std::vector<FrontendConfirmedLandmark> & confirmed,
    double traveled_m,
    const std::function<bool(const Eigen::Vector2d &)> & expected_visible,
    const std::function<bool(const Eigen::Vector2d &)> & promotion_hold = {})
  {
    FrontendFrameResult result;
    // Travel since the previous processed frame: the unit blind-travel
    // accounting is charged in, whatever the frame/keyframe cadence.
    const double travel_delta = last_processed_traveled_m_ >= 0.0 ?
      std::max(0.0, traveled_m - last_processed_traveled_m_) : 0.0;
    last_processed_traveled_m_ = traveled_m;

    // --- Confirmed-landmark candidates + per-observation ambiguity -------
    struct Candidate
    {
      double chi2;
      std::size_t observation;
      bool is_track;
      std::size_t target;
    };
    std::vector<Candidate> candidates;
    std::vector<bool> ambiguous(observations.size(), false);

    for (std::size_t i = 0; i < observations.size(); ++i) {
      const FrontendObservation & observation = observations[i];
      double best_chi2 = std::numeric_limits<double>::max();
      double second_chi2 = std::numeric_limits<double>::max();
      int best_index = -1;
      int second_index = -1;
      double best_inflation = 0.0;
      double second_inflation = 0.0;

      for (std::size_t j = 0; j < confirmed.size(); ++j) {
        const FrontendConfirmedLandmark & landmark = confirmed[j];
        const Eigen::Vector2d diff = landmark.position - observation.map_point;

        // Drift since last sighting widens the gate (distance-proportional,
        // keyframe-density independent) — this is what lets lap-closure
        // re-association succeed under accumulated odometry error.
        const double meters_unseen =
          std::max(0.0, traveled_m - landmark.last_seen_traveled_m);
        const double inflation = std::min(
          params_.association_max_inflation,
          params_.association_inflation_per_meter * meters_unseen);

        const double gate_radius =
          params_.association_max_distance_m + 3.0 * std::sqrt(inflation);
        if (diff.squaredNorm() > gate_radius * gate_radius) {
          continue;
        }

        const Eigen::Matrix2d innovation_covariance =
          landmark.covariance + observation.map_covariance +
          Eigen::Matrix2d::Identity() * inflation;
        if (!frontend_detail::invertibleInnovation(innovation_covariance)) {
          continue;
        }

        const double chi2 = diff.dot(innovation_covariance.inverse() * diff);
        const bool chi2_ok = chi2 <= params_.association_gate_chi2;
        const bool euclidean_ok =
          diff.squaredNorm() <=
          params_.association_max_distance_m * params_.association_max_distance_m;

        if (chi2 < best_chi2) {
          second_chi2 = best_chi2;
          second_index = best_index;
          second_inflation = best_inflation;
          best_chi2 = chi2;
          best_index = static_cast<int>(j);
          best_inflation = inflation;
        } else if (chi2 < second_chi2) {
          second_chi2 = chi2;
          second_index = static_cast<int>(j);
          second_inflation = inflation;
        }

        if (chi2_ok || euclidean_ok) {
          candidates.push_back(Candidate{chi2, i, false, j});
        }
      }

      // Two confirmed landmarks explain this observation almost equally
      // well. If the rivals are close to EACH OTHER they are a duplicate
      // pair of one physical cone (not a real ambiguity — refusing every
      // revisit near a duplicate starves loop confirmation); otherwise the
      // observation is dropped for the frame. The duplicate-pair separation
      // scales with the drift inflation the gate itself admitted: a twin
      // spawned under N meters of drift sits up to the inflated gate radius
      // from its original, and judging it at the un-inflated radius turned
      // every seam revisit into an "ambiguity" that reset loop confirmation
      // (map never converged, mapping never ended).
      if (best_index >= 0 && second_index >= 0 &&
        second_chi2 <= params_.association_gate_chi2 &&
        std::sqrt(best_chi2) >
        params_.association_ambiguity_ratio * std::sqrt(second_chi2))
      {
        const double rival_separation =
          (confirmed[static_cast<std::size_t>(best_index)].position -
          confirmed[static_cast<std::size_t>(second_index)].position).norm();
        const double duplicate_separation = std::min(
          params_.duplicate_pair_max_separation_m,
          params_.association_max_distance_m +
          3.0 * std::sqrt(std::max(best_inflation, second_inflation)));
        if (rival_separation > duplicate_separation) {
          ambiguous[i] = true;
          result.ambiguous_observations.push_back(i);
        }
      }
    }

    // --- Tentative-track candidates --------------------------------------
    for (std::size_t i = 0; i < observations.size(); ++i) {
      if (ambiguous[i]) {
        continue;
      }
      const FrontendObservation & observation = observations[i];
      const Eigen::Matrix2d observation_covariance =
        leveredCovariance(observation);
      for (std::size_t k = 0; k < tracks_.size(); ++k) {
        const Eigen::Vector2d diff = tracks_[k].position - observation.map_point;
        if (diff.squaredNorm() >
          params_.track_gate_max_distance_m * params_.track_gate_max_distance_m)
        {
          continue;
        }
        const Eigen::Matrix2d innovation_covariance =
          tracks_[k].covariance + observation_covariance;
        if (!frontend_detail::invertibleInnovation(innovation_covariance)) {
          continue;
        }
        const double chi2 = diff.dot(innovation_covariance.inverse() * diff);
        if (chi2 <= params_.track_gate_chi2) {
          candidates.push_back(Candidate{chi2, i, true, k});
        }
      }
    }

    // --- Frame-global greedy assignment (ascending chi2, mutual
    // exclusion): order-independent, and confirmed landmarks compete with
    // tracks on the same footing so a seam observation goes to whichever
    // explains it best.
    std::sort(
      candidates.begin(), candidates.end(),
      [](const Candidate & a, const Candidate & b) {return a.chi2 < b.chi2;});

    std::vector<bool> observation_assigned(observations.size(), false);
    std::vector<bool> landmark_claimed(confirmed.size(), false);
    std::vector<bool> track_hit(tracks_.size(), false);

    for (const Candidate & candidate : candidates) {
      if (observation_assigned[candidate.observation] || ambiguous[candidate.observation]) {
        continue;
      }
      if (candidate.is_track) {
        if (track_hit[candidate.target]) {
          continue;
        }
        track_hit[candidate.target] = true;
        observation_assigned[candidate.observation] = true;
        updateTrack(tracks_[candidate.target], observations[candidate.observation], traveled_m);
        ++result.updated_tracks;
      } else {
        if (landmark_claimed[candidate.target]) {
          continue;
        }
        landmark_claimed[candidate.target] = true;
        observation_assigned[candidate.observation] = true;
        result.confirmed_matches.push_back(
          FrontendConfirmedMatch{candidate.observation, candidate.target});
      }
    }

    // --- Unmatched, unambiguous observations found new tracks. No range or
    // crowd gate: a track is invisible to the map, so a bad founding costs
    // nothing but the frames it takes to die.
    for (std::size_t i = 0; i < observations.size(); ++i) {
      if (observation_assigned[i] || ambiguous[i]) {
        continue;
      }
      foundTrack(observations[i], traveled_m);
      ++result.new_tracks;
    }
    // Tracks founded this frame were just observed: they enter maintenance
    // as hits, never as misses.
    track_hit.resize(tracks_.size(), true);

    // --- Track maintenance: misses, deaths, promotions -------------------
    std::vector<TentativeTrack> survivors;
    survivors.reserve(tracks_.size());
    for (std::size_t k = 0; k < tracks_.size(); ++k) {
      TentativeTrack & track = tracks_[k];

      if (!track_hit[k]) {
        if (expected_visible && expected_visible(track.position)) {
          track.blind_travel_m += travel_delta;
        }
        if (track.blind_travel_m >= params_.kill_blind_travel_m) {
          ++result.killed_tracks;
          continue;
        }
      }
      const bool promotable = isPromotable(track);
      // The caller's hold: a converged real cone waiting on the caller (seam
      // pending). It must not age out while it waits, and it promotes on the
      // very frame the hold lifts -- the travel kill below only sees tracks
      // that are still waiting or never converged.
      const bool caller_held =
        promotable && promotion_hold && promotion_hold(track.position);
      if (caller_held) {
        ++result.held_promotions;
        ++result.caller_held_promotions;
      } else if (promotable) {
        bool held = false;
        for (const FrontendConfirmedLandmark & landmark : confirmed) {
          const double stale_travel = traveled_m - landmark.last_seen_traveled_m;
          if (stale_travel <= params_.promote_hold_stale_travel_m) {
            continue;
          }
          // The hold radius scales with the drift the association gate
          // itself models for this landmark: a drift-offset re-sighting of a
          // stale cone sits up to the inflated gate radius away, so holding
          // at the base radius let seam-crossing twins promote 3-5 m from
          // their originals and freeze into the map. Only LAP-OLD stale
          // neighbours inflate (see promote_hold_drift_stale_m) — a fresh or
          // merely FOV-stale neighbour still means two real cones.
          double hold_radius = params_.promote_hold_radius_m;
          if (stale_travel >= params_.promote_hold_drift_stale_m) {
            const double inflation = std::min(
              params_.association_max_inflation,
              params_.association_inflation_per_meter * stale_travel);
            hold_radius += 3.0 * std::sqrt(inflation);
          }
          if ((landmark.position - track.position).norm() <= hold_radius) {
            held = true;
            break;
          }
        }
        if (held) {
          ++result.held_promotions;
        } else {
          result.promotions.push_back(makePromotion(track));
          continue;
        }
      }
      // Unconverged (or stale-twin-held) for too much travel: a stray. A
      // caller-held track is a converged real cone waiting on the caller and
      // is exempt.
      if (!caller_held &&
        traveled_m - track.first_seen_traveled_m > params_.kill_unpromoted_travel_m)
      {
        ++result.killed_tracks;
        continue;
      }

      survivors.push_back(std::move(track));
    }
    tracks_ = std::move(survivors);

    return result;
  }

private:
  // Promotion criteria: enough hits AND a converged position. Promotion
  // sigma is the fused covariance PLUS the correlated lever floor (composed
  // in quadrature; the fused part already contains per-frame lever terms,
  // so this is deliberately conservative). The far tier admits a cone only
  // ever seen at range with its honest, wider sigma after more hits.
  bool isPromotable(const TentativeTrack & track) const
  {
    const double fused_sigma = frontend_detail::maxAxisSigma(track.covariance);
    const double lever_floor =
      track.min_lever_sigma_m == std::numeric_limits<double>::max() ?
      0.0 : track.min_lever_sigma_m;
    const double promotion_sigma =
      std::sqrt(fused_sigma * fused_sigma + lever_floor * lever_floor);
    return (track.hits >= params_.promote_min_hits &&
           promotion_sigma <= params_.promote_max_position_sigma_m) ||
           (track.hits >= params_.promote_far_min_hits &&
           promotion_sigma <= params_.promote_far_max_sigma_m);
  }

  // Hand a track over as a promotion (moves its replay history out).
  static FrontendPromotion makePromotion(TentativeTrack & track)
  {
    FrontendPromotion promotion;
    promotion.position = track.position;
    promotion.covariance = track.covariance;
    promotion.hits = track.hits;
    promotion.color_votes = track.color_votes;
    promotion.color = majorityColor(track.color_votes);
    promotion.observations = std::move(track.pending);
    return promotion;
  }

  Eigen::Matrix2d leveredCovariance(const FrontendObservation & observation) const
  {
    const double lever_sigma = params_.heading_lever_sigma_rad * observation.range_m;
    return observation.map_covariance +
           Eigen::Matrix2d::Identity() * (lever_sigma * lever_sigma);
  }

  void foundTrack(const FrontendObservation & observation, double traveled_m)
  {
    TentativeTrack track;
    track.position = observation.map_point;
    track.covariance = leveredCovariance(observation);
    track.hits = 1;
    track.first_seen_traveled_m = traveled_m;
    track.last_seen_traveled_m = traveled_m;
    track.min_lever_sigma_m = params_.heading_lever_sigma_rad * observation.range_m;
    voteColor(track.color_votes, observation.color);
    track.pending.push_back(
      FrontendPendingObservation{
        observation.pose_graph_id,
        observation.keyframe_measurement,
        observation.keyframe_covariance});
    tracks_.push_back(std::move(track));
  }

  void updateTrack(
    TentativeTrack & track,
    const FrontendObservation & observation,
    double traveled_m)
  {
    const Eigen::Matrix2d observation_covariance = leveredCovariance(observation);
    const Eigen::Matrix2d innovation_covariance =
      track.covariance + observation_covariance;
    if (frontend_detail::invertibleInnovation(innovation_covariance)) {
      const Eigen::Matrix2d gain =
        track.covariance * innovation_covariance.inverse();
      track.position =
        track.position + gain * (observation.map_point - track.position);
      const Eigen::Matrix2d updated =
        (Eigen::Matrix2d::Identity() - gain) * track.covariance;
      track.covariance = (0.5 * (updated + updated.transpose())).eval();
    }
    ++track.hits;
    track.blind_travel_m = 0.0;
    track.last_seen_traveled_m = traveled_m;
    track.min_lever_sigma_m = std::min(
      track.min_lever_sigma_m,
      params_.heading_lever_sigma_rad * observation.range_m);
    voteColor(track.color_votes, observation.color);
    // One replay edge per keyframe: several sightings from the same pose
    // share its error, so stacking them would double-count information.
    // Keep the latest sighting for that pose instead.
    if (!track.pending.empty() &&
      track.pending.back().pose_graph_id == observation.pose_graph_id)
    {
      track.pending.back().keyframe_measurement = observation.keyframe_measurement;
      track.pending.back().keyframe_covariance = observation.keyframe_covariance;
    } else if (track.pending.size() < params_.max_pending_observations) {
      track.pending.push_back(
        FrontendPendingObservation{
          observation.pose_graph_id,
          observation.keyframe_measurement,
          observation.keyframe_covariance});
    }
  }

  static void voteColor(
    std::array<std::uint16_t, kFrontendColorCount> & votes,
    std::uint8_t color)
  {
    if (color >= kFrontendColorCount - 1U) {
      return;  // unknown never votes
    }
    if (votes[color] < std::numeric_limits<std::uint16_t>::max()) {
      ++votes[color];
    }
  }

  static std::uint8_t majorityColor(
    const std::array<std::uint16_t, kFrontendColorCount> & votes)
  {
    std::size_t best = 0U;
    for (std::size_t i = 1U; i < votes.size(); ++i) {
      if (votes[i] > votes[best]) {
        best = i;
      }
    }
    if (votes[best] == 0U) {
      return static_cast<std::uint8_t>(kFrontendColorCount - 1U);
    }
    return static_cast<std::uint8_t>(best);
  }

  FrontendParams params_;
  std::vector<TentativeTrack> tracks_;
  // traveled_m of the previous processFrame call (-1 before the first).
  double last_processed_traveled_m_{-1.0};
};

}  // namespace hyu_localization

#endif  // HYU_LOCALIZATION__TENTATIVE_TRACK_FRONTEND_HPP_
