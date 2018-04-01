//
//  main.cpp
//  Surface Heat Diffuse
//
//  Created by MINGFENWANG on 2018/3/25.
//  Copyright © 2017年 MINGFENWANG. All rights reserved.
//

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>
#include <thread>
#include <iomanip>
#include <float.h>
#include <time.h>
#include <string.h>

#include <sys/types.h>
#include <sys/stat.h>

#include <unistd.h>

#include "Vector.hpp"
#include "raytri.h"
#include "tribox.h"

// helper function
void split_string(const std::string& s,
                  std::vector<std::string>& v,
                  const std::string& c) {
  std::string::size_type pos1, pos2;
  pos2 = s.find(c);
  pos1 = 0;

  while (std::string::npos != pos2) {
    v.push_back(s.substr(pos1, pos2 - pos1));

    pos1 = pos2 + c.size();
    pos2 = s.find(c, pos1);
  }

  if (pos1 != s.length())
    v.push_back(s.substr(pos1));
}

struct Weight {
  int bone;
  float weight;
};

struct Bone_Heat {
  Bone_Heat() {
    static_heats.shrink_to_fit();
    ping_heats.shrink_to_fit();
    pong_heats.shrink_to_fit();
  }
  std::vector<float> static_heats;
  std::vector<float> ping_heats;
  std::vector<float> pong_heats;
};

struct Vertex {
  Vertex(const float x, const float y, const float z) {
    pos.x = x;
    pos.y = y;
    pos.z = z;
    weights.shrink_to_fit();
    neibours.shrink_to_fit();
  }
  void sort_weight() {
    std::sort(weights.begin(), weights.end(), greater_function);
  }
  static bool greater_function(Weight a, Weight b) {
    return (a.weight > b.weight);
  }

  structvec3 pos;
  std::vector<Weight> weights;
  std::vector<int> neibours;
  Bone_Heat bone_heat;
};

struct Triangle {
  Triangle(const int v0, const int v1, const int v2) {
    v[0] = v0;
    v[1] = v1;
    v[2] = v2;
  }
  int v[3];
};

struct Bone {
  Bone(const std::string _name,
       const structvec3& _head,
       const structvec3& _tail)
      : name(_name), head(_head), tail(_tail) {}
  std::string name;
  structvec3 head;
  structvec3 tail;
};

struct Bone_Point {
  Bone_Point(const int _index, const structvec3& _pos)
      : index(_index), pos(_pos), darkness(0.0f), radius(1.0f) {}
  int index;
  structvec3 pos;
  float darkness;
  float radius;
};

// uniform voxel
struct Voxel {
  Voxel() {
    vertices.shrink_to_fit();
    triangles.shrink_to_fit();
  }
  std::vector<int> vertices;
  std::vector<int> triangles;
};

// 3d voxel grid class
class VoxelGrid {
 public:
  VoxelGrid(const std::string& _mesh_file,
            const std::string& _bone_file,
            const std::string& _weight_file,
            const int _max_grid_num,
            const int _max_diffuse_loop,
            const int _max_sample_num,
            const int _max_influence,
            const float _max_fall_off) {
    // init parameters
    mesh_file = _mesh_file;
    bone_file = _bone_file;
    weight_file = _weight_file;
    max_grid_num = _max_grid_num;
    max_diffuse_loop = _max_diffuse_loop;
    max_sample_num = _max_sample_num;
    max_influence = _max_influence;
    max_fall_off = _max_fall_off;

    // read mesh data from text file
    read_mesh_from_file(mesh_file);
    shrink_all_vertex_neibours();

    // read bone data from text file
    read_bone_from_file(bone_file);
    // reset heat buffers to bone size
    init_all_vertex_heats();

    std::cout << "Building voxel grid..." << std::endl;
    // init the bound box
    structvec3 bb_min(FLT_MAX, FLT_MAX, FLT_MAX);
    structvec3 bb_max(FLT_MIN, FLT_MIN, FLT_MIN);

    // loop through all triangles
    for (int i = 0; i < static_cast<int>(triangles.size()); i++) {
      // get the bound box
      get_bound_box(triangles[i], bb_min, bb_max);
    }

    // get standard bound box
    structvec3 bb_std = bb_max - bb_min;

    // to contain the triangles, divide the longest by max grid num minus one.
    grid_size =
        std::max(std::max(bb_std.x, bb_std.y), bb_std.z) / (max_grid_num - 1);

    const float inv_grid_size = 1.0f / grid_size;
    // plus all the grid num with one.
    grid_num_x = ceilf(bb_std.x * inv_grid_size) + 1;
    grid_num_y = ceilf(bb_std.y * inv_grid_size) + 1;
    grid_num_z = ceilf(bb_std.z * inv_grid_size) + 1;

    // move down the grid offset with half of the grid size.
    grid_offset = bb_min - structvec3(grid_size, grid_size, grid_size) * 0.5f;

    // prepare to build octrees
    structvec3 grid_min =
        bb_min - structvec3(grid_size, grid_size, grid_size) * 0.5f;
    structvec3 grid_max =
        bb_max + structvec3(grid_size, grid_size, grid_size) * 0.5f;

    float min[3], max[3], cellSize[3];
    min[0] = grid_min.x;
    min[1] = grid_min.y;
    min[2] = grid_min.z;
    max[0] = grid_max.x;
    max[1] = grid_max.y;
    max[2] = grid_max.z;
    cellSize[0] = grid_size;
    cellSize[1] = grid_size;
    cellSize[2] = grid_size;

    // resize the voxels array
    voxels.resize(grid_num_z * grid_num_y * grid_num_x);
    voxels.shrink_to_fit();

    // init ping pong mode
    ping_pong = true;

    // distribute all vertices in the whole grid
    add_all_vertices();
    shrink_all_vertices();

    // distribute all triangles in the whole grid
    add_all_triangles();
    shrink_all_triangles();

    // sample bone segments
    add_all_bones();
  }

  void calculate_all_voxel_darkness() {
    int supported_thread_num = std::thread::hardware_concurrency();
    // at least we can do with one thread
    if (supported_thread_num == 0) {
      supported_thread_num = 1;
    }
    std::cout << "supported thread num: " << supported_thread_num << std::endl;

    std::cout << "Calculating voxels darkness..." << std::endl;
    std::vector<std::thread> threads(supported_thread_num);

    // start all threads
    for (int i = 0; i < supported_thread_num; i++) {
      threads[i] = std::thread(&VoxelGrid::calculate_darkness_in_range, this, i,
                               supported_thread_num);
    }

    // wait for all threads end
    for (int i = 0; i < supported_thread_num; i++) {
      threads[i].join();
    }
  }

  void diffuse_all_heats() {
    int supported_thread_num = std::thread::hardware_concurrency();
    // at least we can do with one thread
    if (supported_thread_num == 0) {
      supported_thread_num = 1;
    }
    std::cout << "supported thread num: " << supported_thread_num << std::endl;

    std::cout << "Diffusing heats..." << std::endl;
    for (int i = 0; i < max_grid_num * max_diffuse_loop; i++) {
      std::cout << "Diffuse pass: " << i << std::endl;

      std::vector<std::thread> threads(supported_thread_num);

      // start all threads
      for (int j = 0; j < supported_thread_num; j++) {
        threads[j] = std::thread(&VoxelGrid::diffuse_vertex_in_range, this, j,
                                 supported_thread_num);
      }

      // wait for all threads end
      for (int j = 0; j < supported_thread_num; j++) {
        threads[j].join();
      }

      // flip ping pong
      ping_pong = !ping_pong;

      // the error is small enough
      if (vertex_heat_standard_error() < 1e-5) {
        break;
      }
    }
  }

  void generate_weight_for_vertices() {
    std::cout << "Generating weight for vertices..." << std::endl;
    for (int i = 0; i < static_cast<int>(vertices.size()); i++) {
      // prepare to fill weights
      vertices[i].weights.resize(bones.size());
      vertices[i].weights.shrink_to_fit();

      // copy heats as vertex weights from its voxel
      if (ping_pong) {
        for (int j = 0; j < static_cast<int>(bones.size()); j++) {
          vertices[i].weights[j].bone = j;
          vertices[i].weights[j].weight = vertices[i].bone_heat.pong_heats[j];
        }
      } else {
        for (int j = 0; j < static_cast<int>(bones.size()); j++) {
          vertices[i].weights[j].bone = j;
          vertices[i].weights[j].weight = vertices[i].bone_heat.ping_heats[j];
        }
      }

      // sort weights in big-to-small order
      vertices[i].sort_weight();
      // remain max influence biggest weights
      vertices[i].weights.resize(
          std::max(std::min(max_influence, static_cast<int>(bones.size())), 0));
      vertices[i].weights.shrink_to_fit();

      // calculate weights sum
      float sum = 0.0f;
      for (int j = 0; j < static_cast<int>(vertices[i].weights.size()); j++) {
        sum += vertices[i].weights[j].weight;
      }

      // make weights sum equals one
      if (sum > 0.0f) {
        for (int j = 0; j < static_cast<int>(vertices[i].weights.size()); j++) {
          vertices[i].weights[j].weight /= sum;
        }
      }
    }
  }

  void export_bone_weights() {
    std::cout << "Exporting bone weights..." << std::endl;
    std::ofstream fout;

    fout.open(weight_file);
    if (!fout) {
      return;
    }

    fout << "# voxel heat diffuse weight export." << std::endl;
    for (int i = 0; i < static_cast<int>(bones.size()); i++) {
      fout << "b," << bones[i].name << std::endl;
    }
    for (int i = 0; i < static_cast<int>(vertices.size()); i++) {
      for (int j = 0; j < static_cast<int>(vertices[i].weights.size()); j++) {
        fout << "w," << i << ',' << vertices[i].weights[j].bone << ','
             << vertices[i].weights[j].weight << std::endl;
      }
    }

    fout.close();
  }

 private:
  void read_mesh_from_file(const std::string& filename) {
    std::ifstream fin;

    fin.open(filename);
    if (!fin) {
      return;
    }

    // read vertices
    float v1, v2, v3;
    std::string line;
    while (std::getline(fin, line)) {
      std::string label;
      if (line.find("v,") == 0) {
        std::vector<std::string> strs;
        split_string(line, strs, ",");
        v1 = atof(strs[1].c_str());
        v2 = atof(strs[2].c_str());
        v3 = atof(strs[3].c_str());
        vertices.push_back(Vertex(v1, v2, v3));
      }
    }
    vertices.shrink_to_fit();

    fin.close();

    fin.open(filename);
    if (!fin) {
      return;
    }

    // read faces
    int f1, f2, f3;
    while (std::getline(fin, line)) {
      std::string label;
      if (line.find("f,") == 0) {
        std::vector<std::string> strs;
        split_string(line, strs, ",");
        for (int i = 0; i < static_cast<int>(strs.size()) - 3; i++) {
          f1 = atoi(strs[1].c_str());
          f2 = atoi(strs[i + 2].c_str());
          f3 = atoi(strs[i + 3].c_str());
          triangles.push_back(Triangle(f1, f2, f3));
          // add indices to vertex's neibours
          std::vector<int>::iterator it;
          // f1
          it = std::find(vertices[f1].neibours.begin(),
                         vertices[f1].neibours.end(), f2);
          if (it == vertices[f1].neibours.end())
            vertices[f1].neibours.push_back(f2);
          it = std::find(vertices[f1].neibours.begin(),
                         vertices[f1].neibours.end(), f3);
          if (it == vertices[f1].neibours.end())
            vertices[f1].neibours.push_back(f3);
          // f2
          it = std::find(vertices[f2].neibours.begin(),
                         vertices[f2].neibours.end(), f1);
          if (it == vertices[f2].neibours.end())
            vertices[f2].neibours.push_back(f1);
          it = std::find(vertices[f2].neibours.begin(),
                         vertices[f2].neibours.end(), f3);
          if (it == vertices[f2].neibours.end())
            vertices[f2].neibours.push_back(f3);
          // f3
          it = std::find(vertices[f3].neibours.begin(),
                         vertices[f3].neibours.end(), f1);
          if (it == vertices[f3].neibours.end())
            vertices[f3].neibours.push_back(f1);
          it = std::find(vertices[f3].neibours.begin(),
                         vertices[f3].neibours.end(), f2);
          if (it == vertices[f3].neibours.end())
            vertices[f3].neibours.push_back(f2);
        }
      }
    }
    triangles.shrink_to_fit();

    fin.close();
  }

  void read_bone_from_file(const std::string& filename) {
    std::ifstream fin;

    fin.open(filename);
    if (!fin) {
      return;
    }

    // read bones
    std::string name;
    structvec3 head, tail;
    std::string line;
    while (std::getline(fin, line)) {
      std::string label;
      if (line.find("b,") == 0) {
        std::vector<std::string> strs;
        split_string(line, strs, ",");
        name = strs[1];
        head.x = atof(strs[2].c_str());
        head.y = atof(strs[3].c_str());
        head.z = atof(strs[4].c_str());
        tail.x = atof(strs[5].c_str());
        tail.y = atof(strs[6].c_str());
        tail.z = atof(strs[7].c_str());
        bones.push_back(Bone(name, head, tail));
      }
    }
    bones.shrink_to_fit();

    fin.close();
  }

  inline bool has_triangle(Voxel& voxel) { return voxel.triangles.size() > 0; }
  inline bool has_vertex(Voxel& voxel) { return voxel.vertices.size() > 0; }

  void get_bound_box(const Triangle& triangle,
                     structvec3& bb_min,
                     structvec3& bb_max) {
    for (int i = 0; i < 3; i++) {
      if (vertices[triangle.v[i]].pos.x < bb_min.x) {
        bb_min.x = vertices[triangle.v[i]].pos.x;
      }
      if (vertices[triangle.v[i]].pos.y < bb_min.y) {
        bb_min.y = vertices[triangle.v[i]].pos.y;
      }
      if (vertices[triangle.v[i]].pos.z < bb_min.z) {
        bb_min.z = vertices[triangle.v[i]].pos.z;
      }
      if (vertices[triangle.v[i]].pos.x > bb_max.x) {
        bb_max.x = vertices[triangle.v[i]].pos.x;
      }
      if (vertices[triangle.v[i]].pos.y > bb_max.y) {
        bb_max.y = vertices[triangle.v[i]].pos.y;
      }
      if (vertices[triangle.v[i]].pos.z > bb_max.z) {
        bb_max.z = vertices[triangle.v[i]].pos.z;
      }
    }
  }

  void add_all_triangles() {
    // loop through all triangles
    for (int i = 0; i < static_cast<int>(triangles.size()); i++) {
      // add triangle
      add_triangle(i);
    }
  }

  void add_triangle(int n) {
    // init the bound box
    structvec3 bb_min(FLT_MAX, FLT_MAX, FLT_MAX);
    structvec3 bb_max(FLT_MIN, FLT_MIN, FLT_MIN);
    // get the bound box
    get_bound_box(triangles[n], bb_min, bb_max);

    const float inv_grid_size = 1.0f / grid_size;
    // get index range
    int min_x = (bb_min.x - grid_offset.x) * inv_grid_size;
    int min_y = (bb_min.y - grid_offset.y) * inv_grid_size;
    int min_z = (bb_min.z - grid_offset.z) * inv_grid_size;
    int max_x = (bb_max.x - grid_offset.x) * inv_grid_size;
    int max_y = (bb_max.y - grid_offset.y) * inv_grid_size;
    int max_z = (bb_max.z - grid_offset.z) * inv_grid_size;

    // loop through in the range
    for (int z = min_z; z <= max_z; z++) {
      for (int y = min_y; y <= max_y; y++) {
        for (int x = min_x; x <= max_x; x++) {
          float boxcenter[3] = {grid_offset.x + (x + 0.5f) * grid_size,
                                grid_offset.y + (y + 0.5f) * grid_size,
                                grid_offset.z + (z + 0.5f) * grid_size};
          float boxhalfsize[3] = {0.5f * grid_size, 0.5f * grid_size,
                                  0.5f * grid_size};
          float triverts[3][3] = {{vertices[triangles[n].v[0]].pos.x,
                                   vertices[triangles[n].v[0]].pos.y,
                                   vertices[triangles[n].v[0]].pos.z},
                                  {vertices[triangles[n].v[1]].pos.x,
                                   vertices[triangles[n].v[1]].pos.y,
                                   vertices[triangles[n].v[1]].pos.z},
                                  {vertices[triangles[n].v[2]].pos.x,
                                   vertices[triangles[n].v[2]].pos.y,
                                   vertices[triangles[n].v[2]].pos.z}};
          // detect collision
          bool touched = (triBoxOverlap(boxcenter, boxhalfsize, triverts) == 1);

          // touched the voxel
          if (touched) {
            int index = z * grid_num_y * grid_num_x + y * grid_num_x + x;
            // add the triangle
            voxels[index].triangles.push_back(n);
          }
        }
      }
    }
  }

  void add_all_bones() {
    // loop through all bones
    for (int i = 0; i < static_cast<int>(bones.size()); i++) {
      // add bone
      add_bone(bones, i);
    }
  }

  void add_bone(const std::vector<Bone>& bones, int bone_index) {
    // one step equal to grid size multiply step scale
    const float step_scale = 0.25f;

    structvec3 ray_origin = bones[bone_index].head;
    structvec3 ray_target = bones[bone_index].tail;

    // ray direction
    structvec3 ray_dir = bones[bone_index].tail - bones[bone_index].head;

    // how long is one step
    structvec3 delta_step = ray_dir.Normalized() * grid_size * step_scale *
                            (float)max_grid_num / 128;

    // tracked ray position
    structvec3 current_position = ray_origin;

    // start ray casting
    while (true) {
      // calculate current voxel indices
      int current_x = (current_position.x - grid_offset.x) / grid_size;
      int current_y = (current_position.y - grid_offset.y) / grid_size;
      int current_z = (current_position.z - grid_offset.z) / grid_size;

      // ray not beyond the voxel grid
      if (!(current_x < 0 || current_x >= grid_num_x || current_y < 0 ||
            current_y >= grid_num_y || current_z < 0 ||
            current_z >= grid_num_z)) {
        // add to bone point vector
        bone_points.push_back(Bone_Point(bone_index, current_position));
      }

      // forward one step
      current_position += delta_step;

      // ray has missed the target voxel, usually ray is ahead of the target
      if ((current_position - ray_origin).SquareLength() >
          (ray_target - ray_origin).SquareLength()) {
        break;
      }
    }
  }

  void shrink_all_vertex_neibours() {
    // Loop through the vertices
    for (int i = 0; i < static_cast<int>(vertices.size()); i++) {
      vertices[i].neibours.shrink_to_fit();
    }
  }

  void add_all_vertices() {
    const float inv_grid_size = 1.0f / grid_size;
    for (int i = 0; i < static_cast<int>(vertices.size()); i++) {
      int x = (vertices[i].pos.x - grid_offset.x) * inv_grid_size;
      int y = (vertices[i].pos.y - grid_offset.y) * inv_grid_size;
      int z = (vertices[i].pos.z - grid_offset.z) * inv_grid_size;
      int index = z * grid_num_y * grid_num_x + y * grid_num_x + x;
      voxels[index].vertices.push_back(i);
    }
  }

  void shrink_all_vertices() {
    // Loop through the voxel grids
    for (int i = 0; i < static_cast<int>(voxels.size()); i++) {
      voxels[i].vertices.shrink_to_fit();
    }
  }

  void shrink_all_triangles() {
    // Loop through the voxel grids
    for (int i = 0; i < static_cast<int>(voxels.size()); i++) {
      voxels[i].triangles.shrink_to_fit();
    }
  }

  void init_all_vertex_heats() {
    // mark vertex voxel darkest
    for (int i = 0; i < static_cast<int>(vertices.size()); i++) {
      vertices[i].bone_heat.static_heats.assign(bones.size(), 0.0f);
      vertices[i].bone_heat.static_heats.shrink_to_fit();
      vertices[i].bone_heat.ping_heats.assign(bones.size(), 0.0f);
      vertices[i].bone_heat.ping_heats.shrink_to_fit();
      vertices[i].bone_heat.pong_heats.assign(bones.size(), 0.0f);
      vertices[i].bone_heat.pong_heats.shrink_to_fit();
    }
  }

  void bone_point_darkness(int index) {
    const float inv_grid_size = 1.0f / grid_size;
    const float inv_rand_max = 1.0f / RAND_MAX;
    float mean_darkness = 0.0f;
    float mean_radius = 0.0f;
    structvec3 ray_origin = bone_points[index].pos;

    // cast many random direction rays
    const int max_outer_probe_count = 16;
    int sample_count = 0;
    // how long does the ray casts
    float max_ray_length =
        grid_size * sqrtf(grid_num_x * grid_num_x + grid_num_y * grid_num_y +
                          grid_num_z * grid_num_z);
    for (int i = 0; i < max_sample_num; i++) {
      // probe rough result to speed up
      if (sample_count == max_outer_probe_count) {
        float prob_darkness = mean_darkness / sample_count;
        // ignore very bright point
        if (prob_darkness < 0.5f) {
          break;
        }
      }
      // ray direction
      structvec3 ray_dir = structvec3((float)rand() * inv_rand_max - 0.5f,
                                      (float)rand() * inv_rand_max - 0.5f,
                                      (float)rand() * inv_rand_max - 0.5f);
      structvec3 ray_target =
          ray_origin + ray_dir.Normalized() * max_ray_length;

      float walk_distance = 0.0f;

      // start ray casting
      float gx0 = (ray_origin.x - grid_offset.x) * inv_grid_size;
      float gy0 = (ray_origin.y - grid_offset.y) * inv_grid_size;
      float gz0 = (ray_origin.z - grid_offset.z) * inv_grid_size;

      float gx1 = (ray_target.x - grid_offset.x) * inv_grid_size;
      float gy1 = (ray_target.y - grid_offset.y) * inv_grid_size;
      float gz1 = (ray_target.z - grid_offset.z) * inv_grid_size;

      int gx0idx = floorf(gx0);
      int gy0idx = floorf(gy0);
      int gz0idx = floorf(gz0);

      int gx1idx = floorf(gx1);
      int gy1idx = floorf(gy1);
      int gz1idx = floorf(gz1);

      int sx = (gx1idx > gx0idx ? 1 : gx1idx < gx0idx ? -1 : 0);
      int sy = (gy1idx > gy0idx ? 1 : gy1idx < gy0idx ? -1 : 0);
      int sz = (gz1idx > gz0idx ? 1 : gz1idx < gz0idx ? -1 : 0);

      int gx = gx0idx;
      int gy = gy0idx;
      int gz = gz0idx;

      // Planes for each axis that we will next cross
      int gxp = gx0idx + (gx1idx > gx0idx ? 1 : 0);
      int gyp = gy0idx + (gy1idx > gy0idx ? 1 : 0);
      int gzp = gz0idx + (gz1idx > gz0idx ? 1 : 0);

      // Only used for multiplying up the error margins
      float vx = (gx1 == gx0 ? 1 : gx1 - gx0);
      float vy = (gy1 == gy0 ? 1 : gy1 - gy0);
      float vz = (gz1 == gz0 ? 1 : gz1 - gz0);

      // Error is normalized to vx * vy * vz so we only have to multiply up
      float vxvy = vx * vy;
      float vxvz = vx * vz;
      float vyvz = vy * vz;

      // Error from the next plane accumulators, scaled up by vx*vy*vz
      // gx0 + vx * rx == gxp
      // vx * rx == gxp - gx0
      // rx == (gxp - gx0) / vx
      float errx = (gxp - gx0) * vyvz;
      float erry = (gyp - gy0) * vxvz;
      float errz = (gzp - gz0) * vxvy;

      float derrx = sx * vyvz;
      float derry = sy * vxvz;
      float derrz = sz * vxvy;

      // tracked voxel indices
      int current_x, current_y, current_z;
      do {
        current_x = gx;
        current_y = gy;
        current_z = gz;

        // ray has beyond the voxel grid
        if (current_x < 0 || current_x >= grid_num_x || current_y < 0 ||
            current_y >= grid_num_y || current_z < 0 ||
            current_z >= grid_num_z) {
          break;
        }

        // get current index
        int current_index = current_z * grid_num_y * grid_num_x +
                            current_y * grid_num_x + current_x;

        // if the voxel has any triangles
        if (has_triangle(voxels[current_index])) {
          // detect all triangles in the voxel
          float darkness = 0.0f;
          float distance = 0.0f;
          ray_cast_darkness(ray_origin, ray_dir.Normalized(),
                            voxels[current_index], darkness, distance);

          // touched any triangle
          if (darkness != 0.0f) {
            mean_darkness += darkness;
            walk_distance = distance;
            break;
          }
        }

        if (gx == gx1idx && gy == gy1idx && gz == gz1idx)
          break;

        // Which plane do we cross first?
        float xr = fabsf(errx);
        float yr = fabsf(erry);
        float zr = fabsf(errz);

        if (sx != 0 && (sy == 0 || xr < yr) && (sz == 0 || xr < zr)) {
          gx += sx;
          errx += derrx;
        } else if (sy != 0 && (sz == 0 || yr < zr)) {
          gy += sy;
          erry += derry;
        } else if (sz != 0) {
          gz += sz;
          errz += derrz;
        }

      } while (true);

      // how far away the ray has gone
      if (walk_distance > 0.0f) {
        mean_radius += walk_distance;
      } else {
        structvec3 ray_position(grid_offset.x + (current_x + 0.5f) * grid_size,
                                grid_offset.y + (current_y + 0.5f) * grid_size,
                                grid_offset.z + (current_z + 0.5f) * grid_size);
        float radius = (ray_position - ray_origin).Length();
        mean_radius += radius;
      }
      // increase sample number
      sample_count++;
    }
    mean_darkness /= sample_count;
    mean_radius /= sample_count;

    // convert the radius to grid unit
    mean_radius /= grid_size;

    bone_points[index].darkness = mean_darkness;
    bone_points[index].radius = mean_radius;
  }

  void bone_point_glow(int index) {
    // ignore bright bone points
    if (bone_points[index].darkness <= 1.0f)
      return;

    // search vertices in range
    std::vector<int> result = search_vertices_in_range(
        bone_points[index].pos, bone_points[index].radius);

    const float inv_grid_size = 1.0f / grid_size;
    structvec3 ray_origin = bone_points[index].pos;
    for (int i = 0; i < static_cast<int>(result.size()); i++) {
      structvec3 ray_target = vertices[result[i]].pos;
      structvec3 ray_dir = ray_target - ray_origin;
      // start ray casting
      float gx0 = (ray_origin.x - grid_offset.x) * inv_grid_size;
      float gy0 = (ray_origin.y - grid_offset.y) * inv_grid_size;
      float gz0 = (ray_origin.z - grid_offset.z) * inv_grid_size;

      float gx1 = (ray_target.x - grid_offset.x) * inv_grid_size;
      float gy1 = (ray_target.y - grid_offset.y) * inv_grid_size;
      float gz1 = (ray_target.z - grid_offset.z) * inv_grid_size;

      int gx0idx = floorf(gx0);
      int gy0idx = floorf(gy0);
      int gz0idx = floorf(gz0);

      int gx1idx = floorf(gx1);
      int gy1idx = floorf(gy1);
      int gz1idx = floorf(gz1);

      int sx = (gx1idx > gx0idx ? 1 : gx1idx < gx0idx ? -1 : 0);
      int sy = (gy1idx > gy0idx ? 1 : gy1idx < gy0idx ? -1 : 0);
      int sz = (gz1idx > gz0idx ? 1 : gz1idx < gz0idx ? -1 : 0);

      int gx = gx0idx;
      int gy = gy0idx;
      int gz = gz0idx;

      // Planes for each axis that we will next cross
      int gxp = gx0idx + (gx1idx > gx0idx ? 1 : 0);
      int gyp = gy0idx + (gy1idx > gy0idx ? 1 : 0);
      int gzp = gz0idx + (gz1idx > gz0idx ? 1 : 0);

      // Only used for multiplying up the error margins
      float vx = (gx1 == gx0 ? 1 : gx1 - gx0);
      float vy = (gy1 == gy0 ? 1 : gy1 - gy0);
      float vz = (gz1 == gz0 ? 1 : gz1 - gz0);

      // Error is normalized to vx * vy * vz so we only have to
      // multiply
      // up
      float vxvy = vx * vy;
      float vxvz = vx * vz;
      float vyvz = vy * vz;

      // Error from the next plane accumulators, scaled up by vx*vy*vz
      // gx0 + vx * rx == gxp
      // vx * rx == gxp - gx0
      // rx == (gxp - gx0) / vx
      float errx = (gxp - gx0) * vyvz;
      float erry = (gyp - gy0) * vxvz;
      float errz = (gzp - gz0) * vxvy;

      float derrx = sx * vyvz;
      float derry = sy * vxvz;
      float derrz = sz * vxvy;

      // tracked voxel indices
      int current_x, current_y, current_z;
      do {
        current_x = gx;
        current_y = gy;
        current_z = gz;

        // ray has beyond the voxel grid
        if (current_x < 0 || current_x >= grid_num_x || current_y < 0 ||
            current_y >= grid_num_y || current_z < 0 ||
            current_z >= grid_num_z) {
          break;
        }

        // get current index
        int current_index = current_z * grid_num_y * grid_num_x +
                            current_y * grid_num_x + current_x;

        // if the voxel has any triangles
        if (has_triangle(voxels[current_index])) {
          // detect all triangles in the voxel
          int hit = ray_cast_triangle(ray_origin, ray_dir.Normalized(),
                                      voxels[current_index], result[i]);

          // touched any triangle
          if (hit != 0) {
            // hit the target
            if (hit == 2) {
              // convert hit distance to grid unit
              float hit_distance = ray_dir.Length() / grid_size;

              // bone glow also has orientation
              structvec3 bone_dir = bones[bone_points[index].index].tail -
                                    bones[bone_points[index].index].head;
              float light_strength =
                  1.0f - fabsf(ray_dir.Normalized().Dot(bone_dir.Normalized()));

              // how much energy was contributed to the vertex
              float hit_energy = 1.0f / (hit_distance / max_grid_num /
                                         (light_strength + 0.01f));
              // we use power four as the decay ratio
              hit_energy *= hit_energy;
              hit_energy *= hit_energy;

              // generate heat
              vertices[result[i]]
                  .bone_heat.static_heats[bone_points[index].index] +=
                  hit_energy;
            }
            break;
          }
        }

        if (gx == gx1idx && gy == gy1idx && gz == gz1idx)
          break;

        // Which plane do we cross first?
        float xr = fabsf(errx);
        float yr = fabsf(erry);
        float zr = fabsf(errz);

        if (sx != 0 && (sy == 0 || xr < yr) && (sz == 0 || xr < zr)) {
          gx += sx;
          errx += derrx;
        } else if (sy != 0 && (sz == 0 || yr < zr)) {
          gy += sy;
          erry += derry;
        } else if (sz != 0) {
          gz += sz;
          errz += derrz;
        }

      } while (true);
    }
  }

  void calculate_darkness_in_range(const int start, const int block_size) {
    // Loop through in the range.
    for (int i = start; i < static_cast<int>(bone_points.size());
         i += block_size) {
      std::cout << "Proceeding bone point: " << i << " of "
                << bone_points.size() << std::endl;
      bone_point_darkness(i);
      bone_point_glow(i);
    }
  }

  void diffuse_vertex_in_range(const int start, const int block_size) {
    // Loop through in the range.
    for (int i = start; i < static_cast<int>(vertices.size());
         i += block_size) {
      diffuse_vertex(i);
    }
  }

  // get the nearest hit point and the darkness
  inline void ray_cast_darkness(const structvec3& ray_origin,
                                const structvec3& ray_dir,
                                const Voxel& voxel,
                                float& darkness,
                                float& distance) {
    darkness = 0.0f;
    float nearest = FLT_MAX;
    for (int i = 0; i < static_cast<int>(voxel.triangles.size()); i++) {
      float orig[3] = {ray_origin.x, ray_origin.y, ray_origin.z};
      float dir[3] = {ray_dir.x, ray_dir.y, ray_dir.z};
      float vert0[3] = {vertices[triangles[voxel.triangles[i]].v[0]].pos.x,
                        vertices[triangles[voxel.triangles[i]].v[0]].pos.y,
                        vertices[triangles[voxel.triangles[i]].v[0]].pos.z};
      float vert1[3] = {vertices[triangles[voxel.triangles[i]].v[1]].pos.x,
                        vertices[triangles[voxel.triangles[i]].v[1]].pos.y,
                        vertices[triangles[voxel.triangles[i]].v[1]].pos.z};
      float vert2[3] = {vertices[triangles[voxel.triangles[i]].v[2]].pos.x,
                        vertices[triangles[voxel.triangles[i]].v[2]].pos.y,
                        vertices[triangles[voxel.triangles[i]].v[2]].pos.z};
      float t, u, v, det;
      bool touched = (intersect_triangle(orig, dir, vert0, vert1, vert2, &t, &u,
                                         &v, &det) == 1);
      if (touched && t >= 0.0f && t < nearest) {
        nearest = t;
        darkness = (det > 0.0f ? 1.0f : 2.0f);
        distance = t;
      }
    }
  }

  // get the nearest hit point and the darkness
  inline int ray_cast_triangle(const structvec3& ray_origin,
                               const structvec3& ray_dir,
                               const Voxel& voxel,
                               const int vertex_index) {
    float nearest = FLT_MAX;
    float target_t = FLT_MIN;
    bool hit = false;
    for (int i = 0; i < static_cast<int>(voxel.triangles.size()); i++) {
      float orig[3] = {ray_origin.x, ray_origin.y, ray_origin.z};
      float dir[3] = {ray_dir.x, ray_dir.y, ray_dir.z};
      float vert0[3] = {vertices[triangles[voxel.triangles[i]].v[0]].pos.x,
                        vertices[triangles[voxel.triangles[i]].v[0]].pos.y,
                        vertices[triangles[voxel.triangles[i]].v[0]].pos.z};
      float vert1[3] = {vertices[triangles[voxel.triangles[i]].v[1]].pos.x,
                        vertices[triangles[voxel.triangles[i]].v[1]].pos.y,
                        vertices[triangles[voxel.triangles[i]].v[1]].pos.z};
      float vert2[3] = {vertices[triangles[voxel.triangles[i]].v[2]].pos.x,
                        vertices[triangles[voxel.triangles[i]].v[2]].pos.y,
                        vertices[triangles[voxel.triangles[i]].v[2]].pos.z};
      float t, u, v, det;
      bool touched = (intersect_triangle(orig, dir, vert0, vert1, vert2, &t, &u,
                                         &v, &det) == 1);
      if (touched && t >= 0.0f) {
        hit = true;
        if (t <= nearest) {
          // one of the target triangle
          if (triangles[voxel.triangles[i]].v[0] == vertex_index ||
              triangles[voxel.triangles[i]].v[1] == vertex_index ||
              triangles[voxel.triangles[i]].v[2] == vertex_index) {
            target_t = t;
          }
          nearest = t;
        }
      }
    }
    return (hit ? (target_t == nearest ? 2 : 1) : 0);
  }

  std::vector<int> search_vertices_in_range(const structvec3& pos,
                                            const float radius) {
    const float inv_grid_size = 1.0f / grid_size;
    int x = (pos.x - grid_offset.x) * inv_grid_size;
    int y = (pos.y - grid_offset.y) * inv_grid_size;
    int z = (pos.z - grid_offset.z) * inv_grid_size;
    std::vector<int> result;

    // enlarge the radius by one
    for (int zz = z - radius - 1; zz <= z + radius + 1; zz++) {
      if (zz < 0 || zz >= grid_num_z)
        continue;
      for (int yy = y - radius - 1; yy <= y + radius + 1; yy++) {
        if (yy < 0 || yy >= grid_num_y)
          continue;
        for (int xx = x - radius - 1; xx <= x + radius + 1; xx++) {
          if (xx < 0 || xx >= grid_num_x)
            continue;
          int index = zz * grid_num_y * grid_num_x + yy * grid_num_x + xx;
          if (has_vertex(voxels[index])) {
            result.insert(result.end(), voxels[index].vertices.begin(),
                          voxels[index].vertices.end());
          }
        }
      }
    }

    return result;
  }

  float vertex_heat_standard_error() {
    // Loop through the vertices
    float square_error = 0.0f;
    int valid_bone_heat_count = 0;
    for (int i = 0; i < static_cast<int>(vertices.size()); i++) {
      for (int j = 0;
           j < static_cast<int>(vertices[i].bone_heat.ping_heats.size()); j++) {
        // ignore empty bone heat
        if (vertices[i].bone_heat.ping_heats[j] == 0.0f &&
            vertices[i].bone_heat.pong_heats[j] == 0.0f)
          continue;
        float difference = vertices[i].bone_heat.ping_heats[j] -
                           vertices[i].bone_heat.pong_heats[j];
        square_error += difference * difference;
        valid_bone_heat_count++;
      }
    }
    return sqrtf(square_error / valid_bone_heat_count);
  }

  void diffuse_vertex(const int index) {
    // reset current ping-pong buffer to accumulate heat from neibours
    if (ping_pong) {
      memset(&vertices[index].bone_heat.pong_heats.front(), 0,
             vertices[index].bone_heat.pong_heats.size() * sizeof(float));
    } else {
      memset(&vertices[index].bone_heat.ping_heats.front(), 0,
             vertices[index].bone_heat.ping_heats.size() * sizeof(float));
    }

    // unit diffuse decay
    float radius = (float)max_grid_num / 128;
    float unit_diffuse_decay = powf(1.0f - max_fall_off, 1.0f / radius);

    // accumulate heat from neibours
    int neibour_count = 0;
    for (int i = 0; i < static_cast<int>(vertices[index].neibours.size());
         i++) {
      float hit_distance =
          (vertices[index].pos - vertices[vertices[index].neibours[i]].pos)
              .Length() /
          grid_size;

      float hit_energy = powf(unit_diffuse_decay, hit_distance);

      if (ping_pong) {
        for (int j = 0;
             j < static_cast<int>(vertices[index].bone_heat.pong_heats.size());
             j++) {
          vertices[index].bone_heat.pong_heats[j] +=
              vertices[vertices[index].neibours[i]].bone_heat.ping_heats[j] *
              hit_energy;
        }
      } else {
        for (int j = 0;
             j < static_cast<int>(vertices[index].bone_heat.ping_heats.size());
             j++) {
          vertices[index].bone_heat.ping_heats[j] +=
              vertices[vertices[index].neibours[i]].bone_heat.pong_heats[j] *
              hit_energy;
        }
      }
      neibour_count++;
    }

    // heat diffused from any neibour
    if (neibour_count) {
      const float inv_neibour_count = 1.0f / neibour_count;
      // average and decay all heats
      if (ping_pong) {
        for (int j = 0;
             j < static_cast<int>(vertices[index].bone_heat.pong_heats.size());
             j++) {
          vertices[index].bone_heat.pong_heats[j] *= inv_neibour_count;
          vertices[index].bone_heat.pong_heats[j] *= unit_diffuse_decay;
        }
      } else {
        for (int j = 0;
             j < static_cast<int>(vertices[index].bone_heat.ping_heats.size());
             j++) {
          vertices[index].bone_heat.ping_heats[j] *= inv_neibour_count;
          vertices[index].bone_heat.ping_heats[j] *= unit_diffuse_decay;
        }
      }
    }

    // provide energy from static heat cache
    if (ping_pong) {
      for (int j = 0;
           j < static_cast<int>(vertices[index].bone_heat.pong_heats.size());
           j++) {
        vertices[index].bone_heat.pong_heats[j] +=
            vertices[index].bone_heat.static_heats[j];
      }
    } else {
      for (int j = 0;
           j < static_cast<int>(vertices[index].bone_heat.ping_heats.size());
           j++) {
        vertices[index].bone_heat.ping_heats[j] +=
            vertices[index].bone_heat.static_heats[j];
      }
    }
  }

  std::string mesh_file;
  std::string bone_file;
  std::string weight_file;
  int max_grid_num;
  int max_diffuse_loop;
  int max_sample_num;
  int max_influence;
  float max_fall_off;
  int grid_num_x;
  int grid_num_y;
  int grid_num_z;
  float grid_size;
  structvec3 grid_offset;
  std::vector<Voxel> voxels;
  bool ping_pong;
  std::vector<Vertex> vertices;
  std::vector<Triangle> triangles;
  std::vector<Bone> bones;
  std::vector<Bone_Point> bone_points;
};

static void show_usage() {
  std::cout << "Surface Heat Diffuse - Command Line Edition v0.1\n"
            << "http://www.mesh-online.net/\n"
            << "Copyright (c) 2013-2018 Mesh Online. All rights reserved.\n"
            << "Usage: shd <Input Mesh File> <Input Bone File> <Output Weight "
               "File> <Voxel Resolution> <Diffuse Loop> <Sample Rays> "
               "<Influence Bones> <Diffuse Falloff>\n"
            << "Example:\n"
            << "./shd mesh.txt bone.txt weight.txt 128 5 64 4 0.2\n"
            << std::endl;
}

int main(int argc, const char* argv[]) {
  // No enough parameters, show the usage.
  if (argc != 9) {
    show_usage();
    return 1;
  }

  clock_t start, ends;
  start = time(NULL);

  // set random seed
  srand((unsigned int)time(NULL));

  std::string mesh_file = argv[1];
  std::string bone_file = argv[2];
  std::string weight_file = argv[3];
  int max_grid_num = atoi(argv[4]);
  int max_diffuse_loop = atoi(argv[5]);
  int max_sample_num = atoi(argv[6]);
  int max_influence = atoi(argv[7]);
  float max_fall_off = atof(argv[8]);

  VoxelGrid grid(mesh_file, bone_file, weight_file, max_grid_num,
                 max_diffuse_loop, max_sample_num, max_influence, max_fall_off);
  grid.calculate_all_voxel_darkness();
  grid.diffuse_all_heats();
  grid.generate_weight_for_vertices();
  grid.export_bone_weights();

  ends = time(NULL);
  int seconds_elapsed = (int)(ends - start);
  std::cout << "Running Time : " << seconds_elapsed / 60 << " minutes and "
            << seconds_elapsed % 60 << " seconds" << std::endl;

  return 0;
}
