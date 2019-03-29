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

#include <map>

#include <boost/lexical_cast.hpp>
#include <boost/utility/in_place_factory.hpp>
#include <boost/algorithm/string/predicate.hpp>

#include "jsoncpp/as.hpp"

#include "../detail/support.hpp"

#include "parse.hpp"

namespace gltf { namespace v1 {

using detail::common;
using detail::parse;
using detail::namedCommon;

struct Id2Index {
    using Mapping = std::map<std::string, Index>;
    Mapping mapping;
    const char *name;

    Id2Index(const Json::Value &value, const char *name)
        : name(name)
    {
        if (!value.isMember(name)) { return; }

        const auto &elements(value[name]);
        Index index(0);
        for (const auto &member : elements.getMemberNames()) {
            mapping[member] = index++;
        }
    }

    Index operator()(const std::string &id) const {
        const auto fmapping(mapping.find(id));
        if (fmapping == mapping.end()) {
            LOGTHROW(err1, Json::Error)
                << "Entity <" << name << "> of id <"
                << id << "> not found.";
        }
        return fmapping->second;
    }
};

struct IdMapping {
    Id2Index scenes;
    Id2Index nodes;
    Id2Index meshes;
    Id2Index cameras;

    IdMapping(const Json::Value &value)
        : scenes(value, "scenes")
        , nodes(value, "nodes")
        , meshes(value, "meshes")
        , cameras(value, "cameras")
    {}
};

/** Read data from JSON value into the model.
 */
template <typename T>
void parse(boost::optional<T> &dst, const Json::Value &value
           , const char *member);

template <typename T>
void parse(const IdMapping &mapping, boost::optional<T> &dst
           , const Json::Value &value, const char *member);

template <typename T>
void parse(const Id2Index &mapping, boost::optional<T> &dst
           , const Json::Value &value, const char *member);

template <typename T>
bool parseOpt(T &dst, const Json::Value &value, const char *member);

template <typename T>
bool parseOpt(T &dst, const Json::Value &value, const char *member
              , const T &dflt);

template <typename T>
void parse(std::vector<T> &dst, const Json::Value &value
           , const char *member);

template <typename T>
void parse(const IdMapping &mapping, std::vector<T> &dst
           , const Json::Value &value, const char *member);

template <typename T>
void parse(std::vector<T> &dst, const Json::Value &value
           , const char *member, const IdMapping &mapping);

template <typename T>
void parse(std::vector<T> &dst, const Json::Value &value
           , const IdMapping &mapping);

template <typename T>
void parse(const IdMapping &mapping, std::vector<T> &dst
           , const Json::Value &value);

void parse(const Id2Index &mapping, Indices &indices
           , const Json::Value &value)
{
    if (value.type() != Json::arrayValue) {
        LOGTHROW(err1, Json::Error)
            << "Indices are not an array.";
    }

    indices.clear();
    indices.reserve(value.size());
    for (const auto &item : value) {
        indices.push_back(mapping(item.asString()));
    }
}

void parse(const Id2Index &mapping, Indices &indices
           , const Json::Value &value, const char *member)
{
    if (value.isMember(member)) {
        parse(mapping, indices, value[member]);
    } else {
        indices.clear();
    }
}

void parse(Asset &asset, const Json::Value &value)
{
    common(asset, value);
    Json::get(asset.copyright, value, "copyright");
    Json::get(asset.generator, value, "generator");
    parse(asset.version, value["version"]);
    parse(asset.minVersion, value, "minVersion");
}

void parse(const IdMapping &mapping, Scene &scene, const Json::Value &value)
{
    namedCommon(scene, value);
    parse(mapping.nodes, scene.nodes, value, "nodes");
}

void parse(const IdMapping &mapping, Node &node, const Json::Value &value)
{
    namedCommon(node, value);

    std::string cameraId;
    if (Json::getOpt(cameraId, value, "camera")) {
        node.camera = mapping.cameras(cameraId);
    }
    Json::get(node.camera, value, "camera");

    parse(mapping.nodes, node.children, value, "children");
    parse(node.matrix, value, "matrix");
    parse(node.rotation, value, "rotation");
    parse(node.scale, value, "scale");
    parse(node.translation, value, "translation");

    std::string meshId;
    if (Json::getOpt(meshId, value, "mesh")) {
        node.mesh = mapping.meshes(meshId);
    }
}

void parse(const IdMapping &mapping, Primitive &primitive
           , const Json::Value &value)
{
    (void) mapping;

    common(primitive, value);
    const auto &attributes(value["attributes"]);
    for (const auto &name : attributes.getMemberNames()) {
        Index i;
        Json::get(i, attributes, name.c_str());
        primitive.attributes.insert
            (Primitive::Attributes::value_type
             (boost::lexical_cast<AttributeSemantic>(name)
              , i));
    }

    Json::get(primitive.indices, value, "indices");
    // Json::get(primitive.material, value, "material");
    parseOpt(primitive.mode, value, "mode", PrimitiveMode::default_);
}

void parse(const IdMapping &mapping, Mesh &mesh, const Json::Value &value)
{
    namedCommon(mesh, value);
    parse(mesh.primitives, value, "primitives", mapping);
}

void parse(Model &model, const Json::Value &value)
{
    common(model, value);

    // not mandatory since we have some files... sigh
    parseOpt(model.asset, value, "asset");

    IdMapping mapping(value);

    parse(mapping, model.scenes, value, "scenes");
    std::string sceneId;
    if (Json::getOpt(sceneId, value, "scene")) {
        model.scene = mapping.scenes(sceneId);
    }

    parse(mapping, model.nodes, value, "nodes");
    parse(mapping, model.meshes, value, "meshes");

#if 0
    parse(model.samplers, value, "samplers");
    parse(model.images, value, "images");
    parse(model.textures, value, "textures");
    parse(model.materials, value, "materials");
    parse(model.buffers, value, "buffers");
    parse(model.bufferViews, value, "bufferViews");
    parse(model.accessors, value, "accessors");

#endif

    parse(model.extensionsUsed, value, "extensionsUsed");
}

template <typename T>
void parse(boost::optional<T> &dst, const Json::Value &value
           , const char *member)
{
    if (value.isMember(member)) {
        parse(*(dst = boost::in_place()), value[member]);
    }
}

template <typename T>
bool parseOpt(T &dst, const Json::Value &value
              , const char *member)
{
    if (value.isMember(member)) {
        parse(dst, value[member]);
        return true;
    }
    return false;
}

template <typename T>
bool parseOpt(T &dst, const Json::Value &value, const char *member
              , const T &dflt)
{
    if (parseOpt(dst, value, member)) { return true; }
    dst = dflt;
    return false;
}

template <typename T>
void parse(const IdMapping &mapping, boost::optional<T> &dst
           , const Json::Value &value, const char *member)
{
    if (value.isMember(member)) {
        parse(mapping, dst, value[member]);
    } else {
        dst.clear();
    }
}

template <typename T>
void parse(const IdMapping &mapping, std::vector<T> &dst
           , const Json::Value &value, const char *member)
{
    if (value.isMember(member)) {
        parse(mapping, dst, value[member]);
    } else {
        dst.clear();
    }
}

template <typename T>
void parse(const IdMapping &mapping, std::vector<T> &dst
           , const Json::Value &value)
{
    if (value.type() != Json::objectValue) {
        LOGTHROW(err1, Json::Error)
            << "Expected an object.";
    }

    dst.resize(value.size());
    auto idst(dst.begin());
    for (const auto &name : value.getMemberNames()) {
        parse(mapping, *idst++, value[name]);
    }
}

template <typename T>
void parse(std::vector<T> &dst, const Json::Value &value
           , const char *member, const IdMapping &mapping)
{
    if (value.isMember(member)) {
        parse(dst, value[member], mapping);
    } else {
        dst.clear();
    }
}

template <typename T>
void parse(std::vector<T> &dst, const Json::Value &value
           , const IdMapping &mapping)
{
    if (value.type() != Json::arrayValue) {
        LOGTHROW(err1, Json::Error)
            << "Expected an array.";
    }

    dst.resize(value.size());
    auto idst(dst.begin());
    for (const auto &item : value) {
        parse(mapping, *idst++, item);
    }
}

template <typename T>
void parse(std::vector<T> &dst, const Json::Value &value
           , const char *member)
{
    if (value.isMember(member)) {
        parse(dst, value[member]);
    } else {
        dst.clear();
    }
}

} } // namespace gltf::v1
