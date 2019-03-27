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

#include <type_traits>
#include <stack>
#include <iostream>

#include <boost/iostreams/stream_buffer.hpp>
#include <boost/iostreams/device/array.hpp>

#include "dbglog/dbglog.hpp"

#include "math/transform.hpp"

#include "meshloader.hpp"

namespace ublas = boost::numeric::ublas;
namespace bio = boost::iostreams;

namespace gltf {

MeshLoader::~MeshLoader() {}

namespace {

using ArrayBuffer = bio::stream_buffer<bio::array_source> ;
using VertexIndex = std::uint32_t;

template <typename T> const std::vector<T>& getList(const Model &model);
template <typename T> const char* typeName();

#define DEDCLARE_TYPE(Type, Name) \
    template <> const std::vector<Type>&                                \
    getList<Type>(const Model &model) { return model.Name; }            \
    template <> constexpr const char* typeName<Type>() { return #Type; }

DEDCLARE_TYPE(Scene, scenes)
DEDCLARE_TYPE(Node, nodes)
DEDCLARE_TYPE(Mesh, meshes)
DEDCLARE_TYPE(Sampler, samplers)
DEDCLARE_TYPE(Texture, textures)
DEDCLARE_TYPE(Buffer, buffers)
DEDCLARE_TYPE(Image, images)
DEDCLARE_TYPE(BufferView, bufferViews)
DEDCLARE_TYPE(Accessor, accessors)
DEDCLARE_TYPE(Material, materials)
#undef DEDCLARE_TYPE

struct ModelGetter {
    ModelGetter(const Model &model) : model(model) {}
    const Model &model;

    template <typename T> const T& get(Index index) const {
        const auto &list(getList<T>(model));
        if ((index < 0) || (std::size_t(index) >= list.size())) {
            LOGTHROW(err1, std::runtime_error)
                << typeName<T>() << " index " << index << " out of range [0, "
                << list.size() << ").";
        }
        return list[index];
    }

    template <typename T> const T* get(const OptIndex &index) const {
        return index ? &get<T>(*index) : nullptr;
    }

    const Scene& getScene(const OptIndex &useScene) const {
        if (useScene) { return get<Scene>(*useScene); }
        if (model.scene) { return get<Scene>(*model.scene); }
        LOGTHROW(err1, std::runtime_error) << "No usable scene in glTF model.";
        throw;
    }
};

template <typename T>
const char* getName(const T &what)
{
    return what.name ? what.name->c_str() : typeName<T>();
}

using TrafoStack = std::stack<math::Matrix4>;

TrafoStack* applyNodeTrafo(TrafoStack &stack, const Node &node)
{
    if (node.matrix) {
        // TODO: check order
        stack.push(prod(stack.top(), *node.matrix));
        return &stack;
    }

    // TODO: apply T * R * S from node
    if (node.rotation || node.scale || node.translation) {
        LOGTHROW(err1, std::runtime_error)
            << "rotation/scale/translation not implemented yet";
    }

    return nullptr;
}

struct TrafoHolder {
    TrafoStack *trafos;
    TrafoHolder(TrafoStack &trafos, const Node &node)
        : trafos(applyNodeTrafo(trafos, node))
    {}
    ~TrafoHolder() { if (trafos) { trafos->pop(); } }
};

template <typename T>
const T* getBegin(const Accessor &a, const BufferView &bv, const Data &data)
{
    return reinterpret_cast<const T*>
        (data.data() + bv.byteOffset + a.offset);
}

template <typename T>
std::size_t getSkip(const Accessor &a, const BufferView &bv)
{
    // tightly packed? => no skip
    if (!bv.byteStride) { return 0; }

    // stride in scalars
    const auto typedStride(*bv.byteStride / sizeof(T));
    // how many scalars we need to skip to next one
    return typedStride - elementSize(a.type);
}

template <typename T, bool normalized, typename Enable = void>
struct Normalizer;

template <typename T> struct Normalizer<T, false, void> {
    double normalize(T value) const { return value; }
};

// signed
template <typename T>
struct Normalizer<T, true, typename std::enable_if
                  <std::is_signed<T>::value>::type>
{
    double normalize(T value) const {
        if (value >= 0) {
            return value / max;
        }
        return value / min;
    };

    static constexpr double min = -std::numeric_limits<T>::min();
    static constexpr double max = std::numeric_limits<T>::max();
};

// unsigned
template <typename T>
struct Normalizer<T, true, typename std::enable_if
                  <std::is_unsigned<T>::value>::type>
{
    double normalize(T value) const {
        return value / max;
    };

    static constexpr double max = std::numeric_limits<T>::max();
};

template <typename StoredT, bool normalized>
class AttributeReader : private Normalizer<StoredT, normalized> {
public:
    using stored_type = StoredT;

    AttributeReader(const Accessor &a, const BufferView &bv, const Data &data)
        : elementSize_(elementSize(a.type))
        , begin_(getBegin<stored_type>(a, bv, data))
        , skip_(getSkip<stored_type>(a, bv))
        , end_(begin_ + a.count * (elementSize_ + skip_))
    {}

    template <typename F>
    void read(F f) const {
        for (const auto *i(begin_); i != end_; i += skip_) {
            for (std::size_t index(0); index < elementSize_; ++index, ++i) {
                f(this->normalize(*i), index);
            }
        }
    }

private:
    std::size_t elementSize_;

    const stored_type *begin_;
    const std::size_t skip_;
    const stored_type *end_;
};

class AttributeExtractor {
public:
    AttributeExtractor(const Accessor &a, const BufferView &bv
                       , const Data &data)
        : a_(a), bv_(bv), data_(data)
    {}

    /** Calls f(item) for each item.
     */
    template <typename F>
    void read(F f) const {
#define CASE_TYPE(CType, Type, Normalized)                              \
        case ComponentType::CType:                                      \
            return AttributeReader<Type, Normalized>(a_, bv_, data_).read(f)

        if (a_.normalized) {
            switch (a_.componentType) {
                CASE_TYPE(byte, std::int8_t, true);
                CASE_TYPE(ubyte, std::uint8_t, true);
                CASE_TYPE(short_, std::int16_t, true);
                CASE_TYPE(ushort, std::uint16_t, true);
                CASE_TYPE(uint, std::uint32_t, true);
                CASE_TYPE(float_, float, false);
            }
        } else {
            switch (a_.componentType) {
                CASE_TYPE(byte, std::int8_t, false);
                CASE_TYPE(ubyte, std::uint8_t, false);
                CASE_TYPE(short_, std::int16_t, false);
                CASE_TYPE(ushort, std::uint16_t, false);
                CASE_TYPE(uint, std::uint32_t, false);
                CASE_TYPE(float_, float, false);
            }
        }

#undef CASE_TYPE
    }

private:
    const Accessor &a_;
    const BufferView &bv_;
    const Data &data_;
};

template <typename StoredT>
class IndexReader {
public:
    using stored_type = StoredT;

    IndexReader(const Accessor &a, const BufferView &bv, const Data &data)
        : begin_(getBegin<stored_type>(a, bv, data))
        , end_(begin_ + a.count)
    {}

    template <typename F>
    void read(F f) const {
        for (const auto *i(begin_); i != end_; ++i) {
            f(*i);
        }
    }

private:
    const stored_type *begin_;
    const stored_type *end_;
};

class IndexExtractor {
public:
    IndexExtractor(const Accessor &a, const BufferView &bv, const Data &data)
        : a_(a), bv_(bv), data_(data)
    {}

    /** Calls f(item) for each item.
     */
    template <typename F>
    void read(F f) const {
        if (a_.type != AttributeType::scalar) {
            LOGTHROW(err1, std::runtime_error)
                << "Index must be a scalar.";
        }

        if (a_.normalized) {
            LOGTHROW(err1, std::runtime_error)
                << "Index must not be normalized.";
        }

#define CASE_TYPE(CType, Type)                                  \
        case ComponentType::CType:                              \
            return IndexReader<Type>(a_, bv_, data_).read(f)

        switch (a_.componentType) {
            CASE_TYPE(ubyte, std::uint8_t);
            CASE_TYPE(ushort, std::uint16_t);
            CASE_TYPE(uint, std::uint32_t);
        default:
            LOGTHROW(err1, std::runtime_error)
                << "Index must be a unsigned number.";
        }
#undef CASE_TYPE
    }

private:
    const Accessor &a_;
    const BufferView &bv_;
    const Data &data_;
};

const Data& extractData(const ModelGetter &mg, Index index) {
    struct AttributeExtractor : public boost::static_visitor<const Data&> {
        const Data& operator()(const InlineBuffer &buffer) {
            return buffer.data;
        }

        const Data& operator()(const ExternalBuffer&) {
            LOGTHROW(err1, std::runtime_error)
                << "External buffer is not supported (yet).";
            throw;
        }
    } v;

    return boost::apply_visitor(v, mg.get<Buffer>(index));
}

void addImage(MeshLoader &loader, const Data &data
              , const BufferView *bv = nullptr)
{
    const auto begin(data.data() + (bv ? bv->byteOffset : 0));
    const auto end(begin + (bv ? bv->byteLength : data.size()));

    loader.image(DataView(begin, end));

#if 0
    ArrayBuffer ab(begin, end);
    std::istream is(&ab);
    is.exceptions(std::ios::badbit | std::ios::failbit);
    loader.image(is);
#endif
}

void extractImage(MeshLoader &loader, const BufferView &bv
                  , const Buffer &buffer)
{
    struct ImageAttributeExtractor : public boost::static_visitor<void> {
        MeshLoader &loader;
        const BufferView &bv;

        ImageAttributeExtractor(MeshLoader &loader, const BufferView &bv)
            : loader(loader), bv(bv)
        {}

        void operator()(const InlineBuffer &buffer) {
            addImage(loader, buffer.data, &bv);
        }

        void operator()(const ExternalBuffer&) {
            LOGTHROW(err1, std::runtime_error)
                << "External buffer is not supported (yet).";
        }
    } v(loader, bv);

    return boost::apply_visitor(v, buffer);
}

void extractImage(MeshLoader &loader, const ModelGetter &mg, Index index)
{
    struct ImageExtractor : public boost::static_visitor<> {
        MeshLoader &loader;
        const ModelGetter &mg;
        ImageExtractor(MeshLoader &loader, const ModelGetter &mg)
            : loader(loader), mg(mg)
        {}

        void operator()(const InlineImage &image) {
            addImage(loader, image.data);
        }

        void operator()(const ExternalImage &) {
            LOGTHROW(err1, std::runtime_error)
                << "External image not supported (yet)..";
        }

        void operator()(const BufferViewImage &image) {
            const auto &bv(mg.get<BufferView>(image.bufferView));
            const auto &buffer(mg.get<Buffer>(bv.buffer));
            extractImage(loader, bv, buffer);
        }
    } v(loader, mg);

    return boost::apply_visitor(v, mg.get<Image>(index));
}

class Decoder {
public:
    Decoder(MeshLoader &loader, const Model &model
            , const math::Matrix4 &trafo, const OptIndex &useScene)
        : loader_(loader), mg_(model)
    {
        trafos_.push(trafo);

        for (const auto nodeId : mg_.getScene(useScene).nodes) {
            processNode(mg_.get<Node>(nodeId));
        }
    }

private:
    void processNode(const Node &node) {
        TrafoHolder th(trafos_, node);

        if (const auto *mesh = mg_.get<Mesh>(node.mesh)) {
            processMesh(*mesh);
        }

        for (const auto childId : node.children) {
            processNode(mg_.get<Node>(childId));
        }
    }

    void processMesh(const Mesh &mesh) {
        loader_.mesh();
        for (const auto &primitive : mesh.primitives) {
            processPrimitive(primitive);
        }
    }

    void processPrimitive(const Primitive &primitive) {
        // vertices
        const auto vertexCount([&]()
        {
            math::Points3d vertices;
            extractor<AttributeExtractor>
                (primitive, AttributeSemantic::position)
                .read([&vertices](double value, std::size_t index) mutable
            {
                if (!index) { vertices.emplace_back(); }
                vertices.back()(index) = value;
            });

            // apply transformation
            math::transform(trafo(), vertices);

            auto size(vertices.size());
            loader_.vertices(std::move(vertices));
            return size;
        }());

        // are there texture coordinates
        if (const auto *a = accessor(primitive, AttributeSemantic::texCoord0
                                     , std::nothrow))
        {
            // image
            extractTextureImage(primitive);

            math::Points2d tc;
            extractor<AttributeExtractor>(*a).read
                ([&tc](double value, std::size_t index) mutable
            {
                if (!index) { tc.emplace_back(); }
                tc.back()(index) = value;
            });
            loader_.tc(std::move(tc));
        }

        // faces
        if (const auto *a = mg_.get<Accessor>(primitive.indices)) {
            // read indices from accessor
            loader_.faces(faces(primitive.mode, *a));
        } else {
            loader_.faces(faces(primitive.mode, vertexCount));
        }
    }

    const Accessor& accessor(const Primitive &primitive
                             , AttributeSemantic semantic) const
    {
        if (const auto *a = accessor(primitive, semantic, std::nothrow)) {
            return *a;
        }

        LOGTHROW(err1, std::runtime_error)
            << "Mandatory accessor for attribute " << semantic
            << " not found.";
        throw;
    }

    const Accessor* accessor(const Primitive &primitive
                             , AttributeSemantic semantic
                             , const std::nothrow_t&) const
    {
        if (const auto *a = mg_.get<Accessor>(primitive.attribute(semantic))) {
            a->validate(semantic);
            if (!a->bufferView) {
                LOGTHROW(err1, std::runtime_error)
                    << "Accessor without buffer view is unsupported.";
            }

            return a;
        }
        return nullptr;
    }

    template <typename Extractor>
    Extractor extractor(const Accessor &accessor) const {
        // TODO: handle bufferView-less accessor
        if (!accessor.bufferView) {
            LOGTHROW(err2, std::runtime_error)
                << "No buffer view present in accessor.";
        }
        const auto &bv(mg_.get<BufferView>(*accessor.bufferView));
        return Extractor(accessor, bv, extractData(mg_, bv.buffer));
    }

    template <typename Extractor>
    Extractor extractor(const Primitive &primitive
                            , AttributeSemantic semantic) const
    {
        return extractor<Extractor>(accessor(primitive, semantic));
    }

    void extractTextureImage(const Primitive &primitive) const {
        // extract image from texture from material
        const auto *material(mg_.get<Material>(primitive.material));
        if (!material) {
            LOGTHROW(err1, std::runtime_error)
                << "Expected material for texture coordinates.";
        }

        if (!material->pbrMetallicRoughness) {
            LOGTHROW(err1, std::runtime_error)
                << "Expected PBR metallic roughness material for "
                "texture coordinates.";
        }

        if (!material->pbrMetallicRoughness->baseColorTexture) {
            LOGTHROW(err1, std::runtime_error)
                << "Expected base color texture for texture coordinates.";
        }

        const auto &texture
            (mg_.get<Texture>
             (material->pbrMetallicRoughness->baseColorTexture->index));

        // notify loader with image
        extractImage(loader_, mg_, texture.source);
    }

    MeshLoader::Faces faces(PrimitiveMode mode, const Accessor &a) const {
        const auto e(extractor<IndexExtractor>(a));

        MeshLoader::Faces faces;

        switch (mode) {
        case PrimitiveMode::triangles: {
            // 3 consecutive indices form a triangle
            int i(0);
            e.read([&faces, &i](VertexIndex vi) mutable
            {
                if (!i) { faces.emplace_back(); }
                faces.back()[i++] = vi;
                i %= 3;
            });
        } break;

        case PrimitiveMode::triangleStrip: // TODO: implement me
        case PrimitiveMode::triangleFan: // TODO: implement me
        default:
            LOGTHROW(err1, std::runtime_error)
                << "Only triangles are supported..";
        }

        return faces;
    }

    MeshLoader::Faces faces(PrimitiveMode mode, std::size_t vertexCount)
        const
    {
        MeshLoader::Faces faces;

        switch (mode) {
        case PrimitiveMode::triangles: {
            // 3 consecutive indices form a triangle
            faces.resize(vertexCount / 3);
            std::size_t i(0);
            for (auto &face : faces) {
                for (int ii(0); ii < 3; ++ii, ++i) {
                    face[ii] = i;
                }
            }
        } break;

        case PrimitiveMode::triangleStrip: // TODO: implement me
        case PrimitiveMode::triangleFan: // TODO: implement me
        default:
            LOGTHROW(err1, std::runtime_error)
                << "Only triangles are supported..";
        }

        return faces;
    }

    const math::Matrix4& trafo() const { return trafos_.top(); }

    MeshLoader &loader_;
    const ModelGetter mg_;
    TrafoStack trafos_;
};

} // namespace

void decodeMesh(MeshLoader &loader, const Model &model
                , const math::Matrix4 &trafo
                , const OptIndex &useScene)
{
    Decoder(loader, model, trafo, useScene);
}

} // namespace gltf
