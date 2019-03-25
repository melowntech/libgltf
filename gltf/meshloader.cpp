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

#include <stack>
#include <iostream>

#include <boost/iostreams/stream_buffer.hpp>
#include <boost/iostreams/device/array.hpp>

#include "dbglog/dbglog.hpp"

#include "meshloader.hpp"

namespace ublas = boost::numeric::ublas;
namespace bio = boost::iostreams;

namespace gltf {

namespace {

typedef bio::stream_buffer<bio::array_source> ArrayBuffer;


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

template <typename StoredT, typename T = StoredT>
class DataReader {
public:
    using stored_type = StoredT;
    using value_type = T;

    DataReader(const Accessor &a, const BufferView &bv, const Data &data)
        : elementSize_(elementSize(a.type))
        , begin_(getBegin<stored_type>(a, bv, data))
        , skip_(getSkip<stored_type>(a, bv))
        , end_(begin_ + a.count * (elementSize_ + skip_))
    {}

    template <typename F>
    void read(F f) const {
        for (const auto *i(begin_); i != end_; i += skip_) {
            for (std::size_t index(0); index < elementSize_; ++index, ++i) {
                f(value_type(*i), index);
            }
        }
    }

private:
    std::size_t elementSize_;

    const stored_type *begin_;
    const std::size_t skip_;
    const stored_type *end_;
};

class DataExtractor {
public:
    DataExtractor(const Accessor &a, const BufferView &bv, const Data &data)
        : a_(a), bv_(bv), data_(data)
    {}

    /** Calls f(item) for each item.
     */
    template <typename T, typename F>
    void read(F f) const {
        switch (a_.componentType) {
        case ComponentType::byte:
            return DataReader<std::int8_t, T>(a_, bv_, data_).read(f);
        case ComponentType::ubyte:
            return DataReader<std::uint8_t, T>(a_, bv_, data_).read(f);
        case ComponentType::short_:
            return DataReader<std::int16_t, T>(a_, bv_, data_).read(f);
        case ComponentType::ushort:
            return DataReader<std::uint16_t, T>(a_, bv_, data_).read(f);
        case ComponentType::uint:
            return DataReader<std::uint32_t, T>(a_, bv_, data_).read(f);
        case ComponentType::float_:
            return DataReader<float, T>(a_, bv_, data_).read(f);
        }
    }

private:
    const Accessor &a_;
    const BufferView &bv_;
    const Data &data_;
};

const Data& extractData(const ModelGetter &mg, Index index) {
    struct DataExtractor : public boost::static_visitor<const Data&> {
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
    const auto begin
        (reinterpret_cast<const char*>
         (data.data() + (bv ? bv->byteOffset : 0)));
    const auto end(begin + (bv ? bv->byteLength : data.size()));

    ArrayBuffer ab(begin, end);
    std::istream is(&ab);
    is.exceptions(std::ios::badbit | std::ios::failbit);
    loader.image(is);
}

void extractImage(MeshLoader &loader, const BufferView &bv
                  , const Buffer &buffer)
{
    struct ImageDataExtractor : public boost::static_visitor<void> {
        MeshLoader &loader;
        const BufferView &bv;

        ImageDataExtractor(MeshLoader &loader, const BufferView &bv)
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

        LOG(info4) << "Processing model";
        for (const auto nodeId : mg_.getScene(useScene).nodes) {
            processNode(mg_.get<Node>(nodeId));
        }
    }

private:
    void processNode(const Node &node) {
        TrafoHolder th(trafos_, node);

        LOG(info4) << "node: " << getName(node) << " @" << &node;
        if (const auto *mesh = mg_.get<Mesh>(node.mesh)) {
            processMesh(*mesh);
        }

        for (const auto childId : node.children) {
            processNode(mg_.get<Node>(childId));
        }
    }

    void processMesh(const Mesh &mesh) {
        LOG(info4) << "    mesh: " << getName(mesh) << " @" << &mesh;
        loader_.mesh();
        for (const auto &primitive : mesh.primitives) {
            processPrimitive(primitive);
        }
    }

    void processPrimitive(const Primitive &primitive) {
        LOG(info4) << "    primitive @" << &primitive;

        if (primitive.mode != PrimitiveMode::triangles) {
            LOGTHROW(err1, std::runtime_error)
                << "Only triangles are supported..";
        }

        const auto va(vertexExtractor(primitive));
        const auto tca(tcExtractor(primitive));

#if 0
        {
            math::Points3d vertices;
            va.read<double>([&vertices]
                            (double value, std::size_t index) mutable
            {
                if (!index) { vertices.emplace_back(); }
                vertices.back()(index) = value;
            });
            loader_.vertices(vertices);
        }
#endif

        // TODO: traverse buffer
        (void) tca;
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

    DataExtractor extractor(const Accessor &accessor) const {
        const auto bv(mg_.get<BufferView>(*accessor.bufferView));
        return DataExtractor(accessor, bv, extractData(mg_, bv.buffer));
    }

    DataExtractor vertexExtractor(const Primitive &primitive) const {
        return extractor(accessor(primitive, AttributeSemantic::position));
    }

    DataExtractor tcExtractor(const Primitive &primitive) const {
        // get data extractor
        auto e(extractor(accessor(primitive, AttributeSemantic::texCoord0)));

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

        return e;
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
