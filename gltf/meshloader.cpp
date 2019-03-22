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

#include "dbglog/dbglog.hpp"

#include "meshloader.hpp"

namespace ublas = boost::numeric::ublas;

namespace gltf {

namespace {

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
        loader_.newMesh();
        for (const auto &primitive : mesh.primitives) {
            processPrimitive(primitive);
        }
    }

    void processPrimitive(const Primitive &primitive) {
        LOG(info4) << "    primitive @" << &primitive;

        if (primitive.mode != PrimitiveMode::triangles) {
            LOGTHROW(err1, std::runtime_error)
                << "Only triangles are supported. Sorry.";
        }

        const auto &pa(accessor(primitive, AttributeSemantic::position));
        const auto *tca(accessor(primitive, AttributeSemantic::texCoord0
                                 , std::nothrow));

        (void) pa;
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
        const auto *a(mg_.get<Accessor>(primitive.attribute(semantic)));
        if (a) { a->validate(semantic); }
        return a;
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
