/**
 * Copyright (c) 2019 Melown Technologies SE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * *  Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef gltf_gltf_hpp_included_
#define gltf_gltf_hpp_included_

#include <iosfwd>
#include <stdexcept>
#include <vector>

#include <boost/any.hpp>
#include <boost/optional.hpp>
#include <boost/variant.hpp>
#include <boost/align/aligned_allocator.hpp>

#include "utility/enum-io.hpp"
#include "utility/openmp.hpp"

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

using Index = int;
using Indices = std::vector<int>;
using OptIndex = boost::optional<Index>;
using OptString = boost::optional<std::string> ;
using ExtensionList = std::vector<std::string>;
using Extension = boost::any;
using Extensions = std::map<std::string, Extension>;

// allocate data on with 4-byte alignment to allow direct data read
using DataAllocator = boost::alignment::aligned_allocator<std::uint8_t, 4>;
typedef std::vector<std::uint8_t, DataAllocator> Data;

/** Retuns boost::any set to empty JSON object.
 */
boost::any emptyObject();

/** Returns matrix that transforms from Z-up coordinate system to Y-up
 *  coordinate system (used in glTF).
 */
math::Matrix4 zup2yup();

/** Returns matrix that transforms from Y-up coordinate system (used in glTF) to
 *  Z-up coordinate system.
 */
math::Matrix4 yup2zup();

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
                      ((tangent)("TANGENT"))
                      ((texCoord0)("TEXCOORD_0"))
                      ((texCoord1)("TEXCOORD_1"))
                      ((color0)("COLOR_0"))
                      ((joints0)("JOINTS_0"))
                      ((weights0)("WEIGHTS_0"))
                      )

enum class PrimitiveMode {
    points = 0
    , lines = 1
    , lineLoop = 2
    , lineStrip = 3
    , triangles = 4
    , triangleStrip = 5
    , triangleFan = 6

    , default_ = triangles
};

enum class Target {
    arrayBuffer = 34962
    , elementArrayBuffer = 34963
};

enum class WrappingMode {
    repeat = 10497
    , clamp = 33071
    , mirrored = 33648

    , default_ = repeat
};

struct CommonBase {
    Extensions extensions;
    boost::any extras;
};

struct NamedCommonBase : CommonBase {
    OptString name;
};

struct InlineBuffer : NamedCommonBase {
    Data data;

    void set(const Data &data) { this->data = data; }
    void set(Data &&data) { this->data = std::move(data); }
};

struct ExternalBuffer : NamedCommonBase {
    OptString uri;
    std::size_t byteLength;

    ExternalBuffer() : byteLength() {}
};

using Buffer = boost::variant<InlineBuffer, ExternalBuffer>;
using Buffers = std::vector<Buffer>;

struct BufferView : NamedCommonBase {
    Index buffer;
    std::size_t byteOffset;
    std::size_t byteLength;
    boost::optional<std::size_t> byteStride;
    boost::optional<Target> target;

    BufferView(Index buffer = 0, std::size_t byteLength = 0)
        : buffer(buffer), byteOffset(0), byteLength(byteLength) {}

    using list = std::vector<BufferView>;
};

using ComponentValue = boost::variant<int, double>;
using ComponentValues = std::vector<ComponentValue>;

struct Accessor : NamedCommonBase {
    OptIndex bufferView;
    std::size_t offset;
    ComponentType componentType;
    bool normalized = false;
    std::size_t count;
    AttributeType type;
    ComponentValues min;
    ComponentValues max;

    using list = std::vector<Accessor>;

    Accessor(ComponentType componentType = ComponentType::byte
             , std::size_t count = 0
             , AttributeType type = AttributeType::scalar)
        : offset(), componentType(componentType), count(count), type(type)
    {}

    /** Is this accessor valid for given attribute semantic?
     */
    void validate(AttributeSemantic semantic) const;
};

struct Scene : NamedCommonBase {
    Indices nodes;

    using list = std::vector<Scene>;
};

struct Node : NamedCommonBase {
    OptIndex camera;
    Indices children;
    boost::optional<math::Matrix4> matrix; // serialize as column major!
    boost::optional<math::Point4d> rotation;
    boost::optional<math::Point3d> scale;
    boost::optional<math::Point3d> translation;
    OptIndex mesh;

    using list = std::vector<Node>;
};

struct Primitive : CommonBase {
    using Attributes = std::map<AttributeSemantic, Index>;
    Attributes attributes;
    OptIndex indices;
    OptIndex material;
    PrimitiveMode mode = PrimitiveMode::default_;
    Indices targets;

    using list = std::vector<Primitive>;

    OptIndex attribute(AttributeSemantic semantic) const;
};

struct Mesh : NamedCommonBase {
    Primitive::list primitives;

    using list = std::vector<Mesh>;
};

struct Texture : NamedCommonBase {
    Index sampler;
    Index source;

    Texture(Index sampler = 0, Index source = 0)
        : sampler(sampler), source(source)
    {}

    using list = std::vector<Texture>;
};

struct TextureInfo : CommonBase  {
    static constexpr Index defaultTexCoord = 0;

    Index index;
    Index texCoord = defaultTexCoord;
    boost::optional<double> scale;

    TextureInfo(Index index = 0) : index(index) {}
};

struct PbrMetallicRoughness : CommonBase  {
    boost::optional<TextureInfo> baseColorTexture;
    boost::optional<double> metallicFactor;
    boost::optional<double> roughnessFactor;
};

struct Material : NamedCommonBase {
    boost::optional<PbrMetallicRoughness> pbrMetallicRoughness;

    using list = std::vector<Material>;
};

struct InlineImage : NamedCommonBase {
    Data data;
    std::string mimeType;
};

struct ExternalImage : NamedCommonBase {
    std::string uri;
};

struct BufferViewImage : NamedCommonBase {
    Index bufferView;
    std::string mimeType;

    BufferViewImage(Index bufferView = 0) : bufferView(bufferView) {}
};

struct Sampler : NamedCommonBase {
    boost::optional<int> magFilter;
    boost::optional<int> minFilter;
    WrappingMode wrapS = WrappingMode::default_;
    WrappingMode wrapT = WrappingMode::default_;

    using list = std::vector<Sampler>;
};

using Image = boost::variant<InlineImage, ExternalImage, BufferViewImage>;
using Images = std::vector<Image>;

struct Version {
    int major;
    int minor;

    Version(int major = 2, int minor = 0) : major(major), minor(minor) {}
};

struct Asset : CommonBase {
    OptString copyright;
    OptString generator;
    Version version;
    boost::optional<Version> minVersion;
};

struct Model : CommonBase {
    Asset asset;
    Scene::list scenes;
    OptIndex scene;
    Node::list nodes;
    Mesh::list meshes;
    Sampler::list samplers;
    Texture::list textures;
    Images images;
    Buffers buffers;
    BufferView::list bufferViews;
    Accessor::list accessors;
    Material::list materials;

    ExtensionList extensionsUsed;
    ExtensionList extensionsRequired;

    Scene& defaultScene() {
        if (scenes.empty()) { scenes.emplace_back(); }
        if (scene) { return scenes[*scene]; }
        return scenes.front();
    }
};

template <typename T, typename ...Args>
Index add(std::vector<T> &vector, Args &&...args)
{
    Index index;
    UTILITY_OMP(critical(model_add))
    {
        index = vector.size();
        vector.emplace_back(std::forward<Args>(args)...);
    }
    return index;
}

/** Write a model to an output stream.
 */
void write(std::ostream &os, const Model &model);

/** Write a model to an output file.
 */
void write(const boost::filesystem::path &path, const Model &model);

/** Generate GLB file from a model and external files in srcDir.
 */
void glb(const boost::filesystem::path &path, const Model &model
         , const boost::filesystem::path &srcDir);

/** Generate GLB file from a model and external files in srcDir.
 */
void glb(std::ostream &os, const Model &model
         , const boost::filesystem::path &srcDir);

/** Load a model from a GLB file.
 */
Model glb(std::istream &is, const boost::filesystem::path &path);

/** Deprecated, only for compatibility with older clients.
 */
typedef Model GLTF;

// inlines

inline OptIndex Primitive::attribute(AttributeSemantic semantic) const
{
    auto fattributes(attributes.find(semantic));
    if (fattributes == attributes.end()) { return boost::none; }
    return fattributes->second;
}

inline std::size_t elementSize(AttributeType type)
{
    switch (type) {
    case AttributeType::scalar: return 1;
    case AttributeType::vec2: return 2;
    case AttributeType::vec3: return 3;
    case AttributeType::vec4: return 4;
    case AttributeType::mat2: return 4;
    case AttributeType::mat3: return 9;
    case AttributeType::mat4: return 16;
    }
    throw std::logic_error("Invalid attribute type");
}

} // namespace gltf

#endif // gltf_gltf_hpp_included_
