#ifndef vef2gltf_gltf_hpp_included_
#define vef2gltf_gltf_hpp_included_

#include <iosfwd>

#include <vector>
#include <boost/optional.hpp>
#include <boost/variant.hpp>

#include "utility/enum-io.hpp"

#include "math/geometry_core.hpp"

// Please, don't ask...
#ifdef __GNUC__
#  ifdef major
#     undef major
#  endif
#  ifdef minor
#     undef minor
#  endif
#endif

namespace gltf {

typedef int Index;
typedef std::vector<int> Indices;
typedef boost::optional<Index> OptIndex;
typedef boost::optional<std::string> OptString;

UTILITY_GENERATE_ENUM(AttributeType,
                      ((scalar)("SCALAR"))
                      ((vec2)("VEC2"))
                      ((vec3)("VEC3"))
                      ((vec4)("VEC4"))
                      ((mat2)("MAT2"))
                      ((mat3)("MAT3"))
                      ((mat4)("MAT4"))
                      )

enum class ComponentType {
    byte = 5120
    , ubyte = 5121
    , short_ = 5122
    , ushort = 5123
    , uint = 5125
    , float_ = 5126
};

UTILITY_GENERATE_ENUM(AttributeSemantic,
                      ((position)("POSITION"))
                      ((normal)("NORMAL"))
                      ((target)("TARGET"))
                      ((texCoord0)("TEXCOORD_0"))
                      ((texCoord1)("TEXCOORD_1"))
                      ((color0)("COLOR_0"))
                      ((joints0)("JOINTS_0"))
                      )

enum class PrimitiveMode {
    points = 0
    , lines = 1
    , lineLoop = 2
    , lineStrip = 3
    , triangles = 4
    , triangleStrip = 5
    , triangleFan = 6
};

enum class Target {
    arrayBuffer = 34962
    , elementArrayBuffer = 34963
};

struct Buffer {
    OptString uri;
    std::size_t byteLength;
    OptString name;

    Buffer(std::size_t byteLength = 0) : byteLength(byteLength) {}

    typedef std::vector<Buffer> list;
};

struct BufferView {
    Index buffer;
    boost::optional<std::size_t> byteOffset;
    std::size_t byteLength;
    boost::optional<std::size_t> byteStride;
    boost::optional<Target> target;
    OptString name;

    BufferView(Index buffer = 0, std::size_t byteLength = 0)
        : buffer(buffer), byteLength(byteLength) {}

    typedef std::vector<BufferView> list;
};

typedef boost::variant<int, double> ComponentValue;
typedef std::vector<ComponentValue> ComponentValues;

struct Accessor {
    OptIndex bufferView;
    boost::optional<std::size_t> offset;
    ComponentType componentType;
    boost::optional<bool> normalized;
    std::size_t count;
    AttributeType type;
    ComponentValues min;
    ComponentValues max;
    OptString name;

    Accessor(ComponentType componentType = ComponentType::byte
             , std::size_t count = 0
             , AttributeType type = AttributeType::scalar)
        : componentType(componentType), count(count), type(type)
    {}

    typedef std::vector<Accessor> list;
};

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
    typedef std::map<AttributeSemantic, Index> Attributes;
    Attributes attributes;
    OptIndex indices;
    OptIndex material;
    boost::optional<PrimitiveMode> mode;
    Indices targets;

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
    OptString name;

    Texture(Index sampler, Index source)
        : sampler(sampler), source(source)
    {}

    typedef std::vector<Texture> list;
};

struct TextureInfo {
    Index index;
    OptIndex texCoord;
    boost::optional<double> scale;

    TextureInfo(Index index = 0) : index(index) {}
};

struct PbrMetallicRoughness {
    OptString name;
    boost::optional<TextureInfo> baseColorTexture;
};

struct Material {
    OptString name;
    boost::optional<PbrMetallicRoughness> pbrMetallicRoughness;

    typedef std::vector<Material> list;
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

struct Sampler {
    boost::optional<int> magFilter;
    boost::optional<int> minFilter;
    boost::optional<int> wrapS;
    boost::optional<int> wrapT;
    OptString name;

    typedef std::vector<Sampler> list;
};

typedef boost::variant<InlineImage, ReferencedImage, BufferViewImage> Image;
typedef std::vector<Image> Images;

struct Version {
    int major;
    int minor;

    Version(int major = 2, int minor = 0) : major(major), minor(minor) {}
 };

struct Asset {
    OptString copyright;
    OptString generator;
    Version version;
    boost::optional<Version> minVersion;
};

struct GLTF {
    Asset asset;
    Scene::list scenes;
    OptIndex scene;
    Node::list nodes;
    Mesh::list meshes;
    Sampler::list samplers;
    Texture::list textures;
    Images images;
    Buffer::list buffers;
    BufferView::list bufferViews;
    Accessor::list accessors;
    Material::list materials;

    Scene& defaultScene() {
        if (scenes.empty()) { scenes.emplace_back(); }
        if (scene) { return scenes[*scene]; }
        return scenes.front();
    }
};

template <typename T>
class IndexedValue {
public:
    IndexedValue(Index index, T &value)
        : index_(index), value_(&value)
    {}

    T& operator*() { return *value_; }
    const T& operator*() const { return *value_; }
    T* operator->() { return value_; }
    const T* operator->() const { return value_; }

    Index index() const { return index_; }
    operator Index() const { return index_; }

private:
    Index index_;
    T *value_;
};

template <typename T, typename ...Args>
IndexedValue<T> add(std::vector<T> &vector, Args &&...args)
{
    auto index(vector.size());
    vector.emplace_back(std::forward<Args>(args)...);
    return IndexedValue<T>(index, vector.back());
}

template <typename Variant, typename T, typename ...Args>
IndexedValue<Variant> add(std::vector<T> &vector, Args &&...args)
{
    auto index(vector.size());
    vector.push_back(Variant(std::forward<Args>(args)...));
    return IndexedValue<Variant>(index, boost::get<Variant>(vector.back()));
}

void write(std::ostream &os, const GLTF &gltf);
void write(const boost::filesystem::path &path, const GLTF &gltf);

} // namespace gltf

#endif // vef2gltf_gltf_hpp_included_
