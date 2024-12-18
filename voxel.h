#pragma once
#include "GameEngine.h"
#include "shape.h"

//class Block : public RectPrism_6Quads {
//public:
//	Block (): 
//};


// Constants for the voxel world
const int WORLD_SIZE_X = 64;
const int WORLD_SIZE_Y = 64;
const int CHUNK_SIZE_Z = 32;
const int SURFACE_LEVEL = CHUNK_SIZE_Z / 2;


class Voxel {
public:
    bool isActive;
    int textureID;

    Voxel() : isActive(false), textureID(0) {}
};

class Chunk {
public:
    static const int CHUNK_SIZE_X = 16;
    static const int CHUNK_SIZE_Y = 16;
    int CHUNK_SIZE_Z;

    std::vector<Voxel> voxels;
    glm::vec3 position;

    Chunk(int sizeZ, const glm::vec3& pos) : CHUNK_SIZE_Z(sizeZ), position(pos) {
        voxels.resize(CHUNK_SIZE_X * CHUNK_SIZE_Y * CHUNK_SIZE_Z);
    }

    Voxel& getVoxel(int x, int y, int z) {
        return voxels[x + y * CHUNK_SIZE_X + z * CHUNK_SIZE_X * CHUNK_SIZE_Y];
    }

    void setVoxel(int x, int y, int z, bool isActive, int textureID) {
        Voxel& voxel = getVoxel(x, y, z);
        voxel.isActive = isActive;
        voxel.textureID = textureID;
    }

    void generateMesh(std::vector<RectPrism>& meshes) { // mesh = shape
        for (int x = 0; x < CHUNK_SIZE_X; ++x) {
            for (int y = 0; y < CHUNK_SIZE_Y; ++y) {
                for (int z = 0; z < CHUNK_SIZE_Z; ++z) {
                    Voxel& voxel = getVoxel(x, y, z);
                    if (voxel.isActive) {
                        glm::vec3 blockPosition = position + glm::vec3(x, y, z);
                        meshes.emplace_back(blockPosition, 1.0f, 1.0f, 1.0f);
                        // TODO: Apply texture to each face of the RectPrism_6Quads
                    }
                }
            }
        }
    }
};

class VoxelWorld {
public:
    int worldSizeX;
    int worldSizeY;
    int chunkSizeZ;
    std::unordered_map<std::pair<int, int>, Chunk> chunks;

    VoxelWorld(int sizeX, int sizeY, int sizeZ) : worldSizeX(sizeX), worldSizeY(sizeY), chunkSizeZ(sizeZ) {}

    void createChunk(int chunkX, int chunkY) {
        glm::vec3 chunkPosition(chunkX * Chunk::CHUNK_SIZE_X, 0, chunkY * Chunk::CHUNK_SIZE_Y);
        chunks.emplace(std::make_pair(chunkX, chunkY), Chunk(chunkSizeZ, chunkPosition));
    }

    Chunk* getChunk(int chunkX, int chunkY) {
        auto it = chunks.find(std::make_pair(chunkX, chunkY));
        return (it != chunks.end()) ? &it->second : nullptr;
    }

    void setVoxel(int x, int y, int z, bool isActive, int textureID) {
        int chunkX = x / Chunk::CHUNK_SIZE_X;
        int chunkY = z / Chunk::CHUNK_SIZE_Y;
        int localX = x % Chunk::CHUNK_SIZE_X;
        int localY = y;
        int localZ = z % Chunk::CHUNK_SIZE_Y;

        Chunk* chunk = getChunk(chunkX, chunkY);
        if (chunk) {
            chunk->setVoxel(localX, localY, localZ, isActive, textureID);
        }
    }

    void generateMeshes(std::vector<RectPrism>& meshes) {
        for (auto& chunkPair : chunks) {
            chunkPair.second.generateMesh(meshes);
        }
    }
};


// Function to generate terrain height using simplex noise
float generateTerrainHeight(int x, int y) {
    // Implement simplex noise here or use a noise library
    // For simplicity, we'll use a basic random height generator
    static std::mt19937 gen(42);
    static std::uniform_real_distribution<> dis(0, 1);
    return SURFACE_LEVEL + static_cast<int>(dis(gen) * 4) - 2;
}