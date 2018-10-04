#ifndef vef2gltf_gltf_hpp_included_
#define vef2gltf_gltf_hpp_included_

#include <iosfwd>

#include <vector>
#include <boost/optional.hpp>
#include <boost/variant.hpp>

#include "math/geometry_core.hpp"

namespace gltf {

typedef int Index;
typedef std::vector<int> Indices;
typedef boost::optional<Index> OptIndex;
typedef boost::optional<std::string> OptString;

struct Scene {
    OptString name;
    Indices nodes;

    typedef std::vector<Scene> list;
};

struct Node {
    OptString name;
    OptIndex camera;
    Indices children;
    boost::optional<math::Matrix4> matrix; // serialize as column major!
    OptIndex mesh;

    typedef std::vector<Node> list;
};

struct Primitive {
    typedef std::vector<Primitive> list;
};

struct Mesh {
    OptString name;
    Primitive::list primitives;

    typedef std::vector<Mesh> list;
};

struct Texture {
    Index sampler;
    Index source;

    Texture(Index sampler, Index source)
        : sampler(sampler), source(source)
    {}

    typedef std::vector<Texture> list;
};

struct InlineImage {
    std::vector<unsigned char> data;
    std::string mimeType;
};

struct ReferencedImage {
    std::string uri;
};

struct BufferViewImage {
    Index bufferView;
    std::string mimeType;

    BufferViewImage(Index bufferView) : bufferView(bufferView) {}
};

typedef boost::variant<InlineImage, ReferencedImage, BufferViewImage> Image;
typedef std::vector<Image> Images;

struct Assets {
    Scene::list scenes;
    OptIndex scene;
    Node::list nodes;
    Mesh::list meshes;
    Texture::list textures;
    Images images;

    Assets() {
        scenes.emplace_back();
    }

    Scene& defaultScene() {
        if (scene) { return scenes[*scene]; }
        return scenes.front();
    }

    const Scene& defaultScene() const {
        if (scene) { return scenes[*scene]; }
        return scenes.front();
    }
};

void write(std::ostream &os, const Assets &assets);
void write(const boost::filesystem::path &path, const Assets &assets);

} // namespace gltf

#endif // vef2gltf_gltf_hpp_included_
