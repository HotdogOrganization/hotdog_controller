#ifndef VISUALIZATION_DATA_HPP_
#define VISUALIZATION_DATA_HPP_

#define VISUALIZATION_MAX_PATH_POINTS 2000
#define VISUALIZATION_MAX_PATHS 10
#define VISUALIZATION_MAX_ITEMS 10000

#define VISUALIZATION_MAX_MESHES 5
#define VISUALIZATION_MAX_MESH_GRID 150

#include "cpp_types.hpp"

/**
 * @brief Debugging sphere
 *
 */
struct SphereVisualization {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Vec3< float > position;
    Vec4< float > color;
    double        radius;
};

/**
 * @brief Debugging box
 *
 */
struct BlockVisualization {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Vec3< float > dimension;
    Vec3< float > corner_position;
    Vec3< float > rpy;
    Vec4< float > color;
};

/**
 * @brief Debugging arrow
 *
 */
struct ArrowVisualization {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Vec3< float > base_position;
    Vec3< float > direction;
    Vec4< float > color;
    float         head_width;
    float         head_length;
    float         shaft_width;
};

/**
 * @brief Debugging robot (draws the same type of robot as currently simulating)
 */
struct Hotdog2Visualization {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Vec12< float > q;
    Quat< float >  quat;
    Vec3< float >  p;
    Vec4< float >  color;
};

/**
 * @brief Debugging "path"
 */
struct PathVisualization {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    size_t        num_points = 0;
    Vec4< float > color;
    Vec3< float > position[ VISUALIZATION_MAX_PATH_POINTS ];
    void          clear() {
        num_points = 0;
    }
};

/**
 * @brief Debugging Cone
 */
struct ConeVisualization {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Vec3< float > point_position;
    Vec3< float > direction;
    Vec4< float > color;
    double        radius;
};

/**
 * @brief Mesh Visualization
 */
struct MeshVisualization {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Vec3< float >                                                                    left_corner;
    Eigen::Matrix< float, VISUALIZATION_MAX_MESH_GRID, VISUALIZATION_MAX_MESH_GRID > height_map;

    int rows, cols;

    float grid_size;
    float height_max;
    float height_min;
};

/**
 * @brief Collection of all debugging data
 */
struct VisualizationData {
    size_t              num_paths = 0, num_arrows = 0, num_cones = 0, num_spheres = 0, num_blocks = 0, num_meshes = 0;
    SphereVisualization spheres[ VISUALIZATION_MAX_ITEMS ];
    BlockVisualization  blocks[ VISUALIZATION_MAX_ITEMS ];
    ArrowVisualization  arrows[ VISUALIZATION_MAX_ITEMS ];
    ConeVisualization   cones[ VISUALIZATION_MAX_ITEMS ];
    PathVisualization   paths[ VISUALIZATION_MAX_PATHS ];
    MeshVisualization   meshes[ VISUALIZATION_MAX_MESHES ];

    /**
     * @brief Remove all debug data
     */
    void clear() {
        num_paths = 0, num_arrows = 0, num_cones = 0, num_spheres = 0, num_blocks = 0;
        num_meshes = 0;
    }

    /**
     * @brief Add a new sphere
     *
     * @return A sphere, or nullptr if there isn't enough room
     */
    SphereVisualization* AddSphere() {
        if ( num_spheres < VISUALIZATION_MAX_ITEMS ) {
            return &spheres[ num_spheres++ ];
        }
        return nullptr;
    }

    /**
     * @brief Add a new box
     *
     * @return A box, or nullptr if there isn't enough room
     */
    BlockVisualization* AddBlock() {
        if ( num_blocks < VISUALIZATION_MAX_ITEMS ) {
            return &blocks[ num_blocks++ ];
        }
        return nullptr;
    }

    /**
     * @brief Add a new arrow
     *
     * @return An arrow, or nullptr if there isn't enough room
     */
    ArrowVisualization* AddArrow() {
        if ( num_arrows < VISUALIZATION_MAX_ITEMS ) {
            return &arrows[ num_arrows++ ];
        }
        return nullptr;
    }

    /**
     * @brief Add a new cone
     *
     * @return A cone, or nullptr if there isn't enough room
     */
    ConeVisualization* AddCone() {
        if ( num_cones < VISUALIZATION_MAX_ITEMS ) {
            return &cones[ num_cones++ ];
        }
        return nullptr;
    }

    /**
     * @brief Add a new path
     *
     * @return A path, or nullptr if there isn't enough room
     */
    PathVisualization* AddPath() {
        if ( num_paths < VISUALIZATION_MAX_PATHS ) {
            auto* path = &paths[ num_paths++ ];
            path->clear();
            return path;
        }
        return nullptr;
    }

    /**
     * @brief Add a new Mesh
     *
     * @return A mesh, or nullptr if there isn't enough room
     */
    MeshVisualization* AddMesh() {
        if ( num_paths < VISUALIZATION_MAX_MESHES ) {
            return &meshes[ num_meshes++ ];
        }
        return nullptr;
    }
};

#endif  // VISUALIZATION_DATA_HPP_
