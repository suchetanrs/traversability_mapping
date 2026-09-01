/* Traversability Mapping - A global and local traversability mapping algorithm.
 * Copyright (C) 2024 Suchetan Saravanan and Damien Vivet
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library.  If not, see
 * <https://www.gnu.org/licenses/>.
 */

/**
 * @file MapDatabase.cpp
 * @brief SQLite persistence for System (saveMap / loadMap).
 *
 * SQLite is only the container: every payload is an explicitly laid out little-endian
 * blob whose format is documented in the schema, so the file stays readable by any
 * sqlite3 client and does not depend on compiler or struct layout.
 *
 * Only the minimal replayable state is stored - lattice, extrinsics, and per keyframe
 * {pose, base-frame cloud}. Partials, the grid and every derived layer regenerate on
 * load through the normal rebin path. Keyframes whose cloud was already dropped
 * (mapping/is_kf_optimization_enabled=false) cannot be replayed, so their frozen
 * partials are stored instead and added to the grid directly.
 */

#include "traversability_mapping/System.hpp"

#include <sqlite3.h>

#include <cstdio>
#include <cstdint>
#include <iostream>
#include <vector>

namespace traversability_mapping
{
namespace
{
    constexpr int kFormatVersion = 1;

    static_assert(sizeof(Eigen::Vector3f) == 12, "cloud blobs assume a packed 3-float point");

    /// @brief One stored partial: cell id, point count, then the nine moments.
    struct PartialRec { std::uint64_t cell, n; double s[9]; };
    static_assert(sizeof(PartialRec) == 88, "PartialRec must be padding-free");

    /// @brief Flatten an Affine3f into its 3x4 row-major float block.
    void poseToBlob(const Eigen::Affine3f &T, float out[12])
    {
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 4; ++c)
                out[r * 4 + c] = T.matrix()(r, c);
    }

    /// @brief Rebuild an Affine3f from a 3x4 row-major float block.
    Eigen::Affine3f blobToPose(const float *in)
    {
        Eigen::Affine3f T = Eigen::Affine3f::Identity();
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 4; ++c)
                T.matrix()(r, c) = in[r * 4 + c];
        return T;
    }

    /// @brief Run a statement that returns no rows, reporting any error.
    bool exec(sqlite3 *db, const char *sql)
    {
        char *err = nullptr;
        if (sqlite3_exec(db, sql, nullptr, nullptr, &err) == SQLITE_OK)
            return true;
        std::cerr << "[System] sqlite: " << (err ? err : "unknown error") << std::endl;
        sqlite3_free(err);
        return false;
    }
}  // namespace

bool System::saveMap(const std::string &path)
{
    std::lock_guard<std::recursive_mutex> lock(localMapMutex_);

    std::remove(path.c_str());  // saveMap always writes a fresh file
    sqlite3 *db = nullptr;
    if (sqlite3_open(path.c_str(), &db) != SQLITE_OK)
    {
        std::cerr << "[System] cannot open " << path << ": " << sqlite3_errmsg(db) << std::endl;
        sqlite3_close(db);
        return false;
    }

    const bool ok = exec(db,
        "CREATE TABLE Meta (version INTEGER, x0 REAL, y0 REAL, res REAL, map_frame TEXT,"
        " active_map INTEGER, Tsv BLOB, Tbs BLOB, Tb_obs BLOB, obs_max REAL, obs_min REAL,"
        " obs_set INTEGER);"
        "CREATE TABLE Map (map_id INTEGER PRIMARY KEY);"
        // The trailing newlines matter: without them each -- comment would swallow the
        // rest of the statement, since the concatenated literal is one single line.
        "CREATE TABLE KeyFrame (\n"
        " id INTEGER PRIMARY KEY,\n"
        " map_id INTEGER NOT NULL,\n"
        " stamp REAL,\n"
        " pose BLOB,       -- 3x4 float, row major (map <- base_footprint)\n"
        " cloud BLOB,      -- n * 3 float32, pruned, base frame; NULL if the cloud was dropped\n"
        " partials BLOB);  -- n * (u64 cell_id, u64 N, 9 double); NULL when cloud is present\n");
    if (!ok || !exec(db, "BEGIN"))
    {
        sqlite3_close(db);
        return false;
    }

    // ---- meta ----
    {
        float tsv[12], tbs[12], tobs[12];
        poseToBlob(Tsv_, tsv);
        poseToBlob(Tbs_, tbs);
        poseToBlob(Tb_obs_, tobs);

        std::uint64_t activeMap = 0;
        for (const auto &kv : localMapsSet_)
            if (kv.second == localMap_)
                activeMap = kv.first;

        sqlite3_stmt *st = nullptr;
        sqlite3_prepare_v2(db, "INSERT INTO Meta VALUES (?,?,?,?,?,?,?,?,?,?,?,?)", -1, &st, nullptr);
        sqlite3_bind_int(st, 1, kFormatVersion);
        sqlite3_bind_double(st, 2, lattice_.x0);
        sqlite3_bind_double(st, 3, lattice_.y0);
        sqlite3_bind_double(st, 4, lattice_.res);
        sqlite3_bind_text(st, 5, mapFrame_.c_str(), -1, SQLITE_STATIC);
        sqlite3_bind_int64(st, 6, static_cast<sqlite3_int64>(activeMap));
        sqlite3_bind_blob(st, 7, tsv, sizeof(tsv), SQLITE_STATIC);
        sqlite3_bind_blob(st, 8, tbs, sizeof(tbs), SQLITE_STATIC);
        sqlite3_bind_blob(st, 9, tobs, sizeof(tobs), SQLITE_STATIC);
        sqlite3_bind_double(st, 10, obstacle_max_range_);
        sqlite3_bind_double(st, 11, obstacle_min_range_);
        sqlite3_bind_int(st, 12, obstacleParamsSet_ ? 1 : 0);
        sqlite3_step(st);
        sqlite3_finalize(st);
    }

    // ---- maps ----
    {
        sqlite3_stmt *st = nullptr;
        sqlite3_prepare_v2(db, "INSERT INTO Map VALUES (?)", -1, &st, nullptr);
        for (const auto &kv : localMapsSet_)
        {
            sqlite3_bind_int64(st, 1, static_cast<sqlite3_int64>(kv.first));
            sqlite3_step(st);
            sqlite3_reset(st);
        }
        sqlite3_finalize(st);
    }

    // ---- keyframes ----
    std::size_t nSaved = 0;
    {
        sqlite3_stmt *st = nullptr;
        sqlite3_prepare_v2(db, "INSERT INTO KeyFrame VALUES (?,?,?,?,?,?)", -1, &st, nullptr);
        for (const auto &kv : keyFramesMap_)
        {
            // Obstacle keyframes are transient by construction (the caller ages them out),
            // and their id band is regenerated from scratch on startup: never persist them.
            if (kv.first > nextObstacleId_)
                continue;
            const auto &kf = kv.second;
            const auto rit = allKeyFramesSet_.find(kv.first);
            if (rit == allKeyFramesSet_.end())
                continue;

            float pose[12];
            poseToBlob(kf->getPose(), pose);

            // A retained cloud replays exactly; a dropped one leaves its partials frozen
            // (recomputeKeyFrame bails on !hasCloud), so those are safe to store as-is.
            std::vector<PartialRec> recs;
            if (!kf->hasCloud())
            {
                recs.reserve(kf->partials().size());
                for (const auto &p : kf->partials())
                    recs.push_back({p.first, p.second.N,
                                    {p.second.sx, p.second.sy, p.second.sz,
                                     p.second.sx2, p.second.sy2, p.second.sz2,
                                     p.second.sxy, p.second.sxz, p.second.syz}});
            }

            sqlite3_bind_int64(st, 1, static_cast<sqlite3_int64>(kv.first));
            sqlite3_bind_int64(st, 2, static_cast<sqlite3_int64>(rit->second));
            sqlite3_bind_double(st, 3, kf->getTimestamp());
            sqlite3_bind_blob(st, 4, pose, sizeof(pose), SQLITE_STATIC);
            if (kf->hasCloud())
            {
                sqlite3_bind_blob(st, 5, kf->cloudBase().data(),
                                  static_cast<int>(kf->cloudBase().size() * sizeof(Eigen::Vector3f)),
                                  SQLITE_STATIC);
                sqlite3_bind_null(st, 6);
            }
            else
            {
                sqlite3_bind_null(st, 5);
                sqlite3_bind_blob(st, 6, recs.data(),
                                  static_cast<int>(recs.size() * sizeof(PartialRec)), SQLITE_STATIC);
            }
            sqlite3_step(st);
            sqlite3_reset(st);
            ++nSaved;
        }
        sqlite3_finalize(st);
    }

    exec(db, "COMMIT");
    sqlite3_close(db);
    std::cout << "[System] saved " << nSaved << " keyframes across " << localMapsSet_.size()
              << " maps to " << path << std::endl;
    return true;
}

bool System::loadMap(const std::string &path, std::function<void()> onUpdate)
{
    sqlite3 *db = nullptr;
    if (sqlite3_open_v2(path.c_str(), &db, SQLITE_OPEN_READONLY, nullptr) != SQLITE_OK)
    {
        std::cerr << "[System] cannot open " << path << ": " << sqlite3_errmsg(db) << std::endl;
        sqlite3_close(db);
        return false;
    }

    // ---- meta: the lattice must land before any map is built, since LocalMap takes its
    //      resolution and cell origin from it.
    {
        sqlite3_stmt *st = nullptr;
        sqlite3_prepare_v2(db, "SELECT * FROM Meta", -1, &st, nullptr);
        if (sqlite3_step(st) != SQLITE_ROW)
        {
            std::cerr << "[System] " << path << " has no Meta row." << std::endl;
            sqlite3_finalize(st);
            sqlite3_close(db);
            return false;
        }
        const int version = sqlite3_column_int(st, 0);
        if (version != kFormatVersion)
        {
            std::cerr << "[System] " << path << " is format version " << version
                      << ", expected " << kFormatVersion << "." << std::endl;
            sqlite3_finalize(st);
            sqlite3_close(db);
            return false;
        }

        std::lock_guard<std::recursive_mutex> lock(localMapMutex_);
        lattice_ = Lattice(sqlite3_column_double(st, 1), sqlite3_column_double(st, 2),
                           sqlite3_column_double(st, 3));
        if (const unsigned char *f = sqlite3_column_text(st, 4))
            mapFrame_ = reinterpret_cast<const char *>(f);
        const std::uint64_t activeMap = static_cast<std::uint64_t>(sqlite3_column_int64(st, 5));
        Tsv_ = blobToPose(static_cast<const float *>(sqlite3_column_blob(st, 6)));
        Tbs_ = blobToPose(static_cast<const float *>(sqlite3_column_blob(st, 7)));
        Tbv_ = Tbs_ * Tsv_;
        Tb_obs_ = blobToPose(static_cast<const float *>(sqlite3_column_blob(st, 8)));
        obstacle_max_range_ = sqlite3_column_double(st, 9);
        obstacle_min_range_ = sqlite3_column_double(st, 10);
        obstacleParamsSet_ = sqlite3_column_int(st, 11) != 0;
        sqlite3_finalize(st);

        // ---- maps ----
        sqlite3_prepare_v2(db, "SELECT map_id FROM Map", -1, &st, nullptr);
        while (sqlite3_step(st) == SQLITE_ROW)
            addNewLocalMap(static_cast<std::uint64_t>(sqlite3_column_int64(st, 0)), onUpdate);
        sqlite3_finalize(st);
        setCurrentMap(activeMap);
    }

    // ---- keyframes ----
    std::size_t nLoaded = 0;
    {
        sqlite3_stmt *st = nullptr;
        sqlite3_prepare_v2(db, "SELECT id, map_id, stamp, pose, cloud, partials FROM KeyFrame"
                               " ORDER BY id", -1, &st, nullptr);
        while (sqlite3_step(st) == SQLITE_ROW)
        {
            const std::uint64_t kfID = static_cast<std::uint64_t>(sqlite3_column_int64(st, 0));
            const std::uint64_t mapID = static_cast<std::uint64_t>(sqlite3_column_int64(st, 1));
            const double stamp = sqlite3_column_double(st, 2);
            const Eigen::Affine3f pose = blobToPose(static_cast<const float *>(sqlite3_column_blob(st, 3)));

            if (const void *blob = sqlite3_column_blob(st, 4))
            {
                // Retained cloud: register it and hand over the pose. The owning map's
                // worker rebins and rebuilds the grid through the normal path.
                const int n = sqlite3_column_bytes(st, 4) / static_cast<int>(sizeof(Eigen::Vector3f));
                const auto *pts = static_cast<const Eigen::Vector3f *>(blob);
                std::vector<Eigen::Vector3f> cloud(pts, pts + n);
                if (!registerKeyFrame(stamp, kfID, mapID, std::move(cloud)))
                    continue;
                updateKeyFrame(kfID, pose);
            }
            else
            {
                // Cloud was dropped before the save: restore the frozen partials straight
                // into the grid, since this keyframe can never be re-binned.
                std::lock_guard<std::recursive_mutex> lock(localMapMutex_);
                auto mapIt = localMapsSet_.find(mapID);
                if (mapIt == localMapsSet_.end() || keyFramesMap_.count(kfID))
                    continue;
                auto kf = std::make_shared<KeyFrame>(kfID, stamp, pose,
                                                     std::vector<Eigen::Vector3f>{}, mapID);
                const int n = sqlite3_column_bytes(st, 5) / static_cast<int>(sizeof(PartialRec));
                const auto *recs = static_cast<const PartialRec *>(sqlite3_column_blob(st, 5));
                for (int i = 0; i < n; ++i)
                {
                    NodeMetaData &m = kf->partials()[recs[i].cell];
                    m.N = static_cast<unsigned int>(recs[i].n);
                    m.sx = recs[i].s[0];  m.sy = recs[i].s[1];  m.sz = recs[i].s[2];
                    m.sx2 = recs[i].s[3]; m.sy2 = recs[i].s[4]; m.sz2 = recs[i].s[5];
                    m.sxy = recs[i].s[6]; m.sxz = recs[i].s[7]; m.syz = recs[i].s[8];
                }
                keyFramesMap_[kfID] = kf;
                allKeyFramesSet_[kfID] = mapID;
                mapIt->second->addRestoredKeyFrame(kf);
            }
            ++nLoaded;
        }
        sqlite3_finalize(st);
    }

    sqlite3_close(db);
    std::cout << "[System] loaded " << nLoaded << " keyframes from " << path << std::endl;
    return true;
}
}  // namespace traversability_mapping
